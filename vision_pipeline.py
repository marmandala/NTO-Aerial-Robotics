#!/usr/bin/env python3
import rospy
import math
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA, String
from geometry_msgs.msg import Point
from clover import srv, long_callback
from skimage.morphology import medial_axis
from collections import deque
import json
from threading import Lock


global_data_lock = Lock()

MIN_JUNCTION_AREA = 1
PRUNE_MIN_LENGTH = 20

# --- constants ---
BRANCH_TIME_WINDOW = 150
MIN_HIT_FREQUENCY = 0.5
MERGE_DIST = 0.7
FREEZE_COUNT = 60

branch_candidates = []

rospy.init_node('vision_pipeline')
rospy.set_param('/vision_pipeline/enabled', False)
bridge = CvBridge()
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)

fx = fy = cx = cy = None

global_map_pub = rospy.Publisher("pipeline_global_map", Marker, queue_size=1)
global_junctions_pub = rospy.Publisher("pipeline_global_junctions", Marker, queue_size=1)
global_mask_pub = rospy.Publisher("pipeline_global_mask", Marker, queue_size=1)

bridge = CvBridge()
pub_mask = rospy.Publisher('/debug/mask', Image, queue_size=1)
pub_skeleton = rospy.Publisher('/debug/skeleton', Image, queue_size=1)
pub_junctions = rospy.Publisher('/debug/junctions', Image, queue_size=1)

tubes_pub = rospy.Publisher("tubes", String, queue_size=1)

global_map_pipe = []
global_map_skeleton = []
global_map_junctions = []

occ_pipe = set()
occ_skeleton = set()
occ_junctions = set()

frame_counter = 0


def point_dist(a, b):
    return math.sqrt((a.x - b.x)**2 + (a.y - b.y)**2 + (a.z - b.z)**2)

def neighbors(skel, y, x, h, w):
    nbrs = []
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dy == 0 and dx == 0:
                continue
            ny, nx = y + dy, x + dx
            if 0 <= ny < h and 0 <= nx < w and skel[ny, nx]:
                nbrs.append((ny, nx))
    return nbrs

def find_junctions_and_degrees(skel_bool):
    skel = skel_bool.astype(np.uint8)
    h, w = skel.shape
    kernel = np.array([[1,1,1],[1,0,1],[1,1,1]], dtype=np.uint8)
    neigh_count = cv2.filter2D(skel, cv2.CV_8U, kernel)
    junction_mask = ((skel == 1) & (neigh_count >= 3)).astype(np.uint8)

    num_labels, labels = cv2.connectedComponents(junction_mask, connectivity=8)
    junctions = []

    for lab in range(1, num_labels):
        comp_mask = (labels == lab)
        comp_coords = np.column_stack(np.nonzero(comp_mask))
        if comp_coords.shape[0] < MIN_JUNCTION_AREA:
            continue

        boundary_neighbors = set()
        for (y,x) in comp_coords:
            for dy in (-1,0,1):
                for dx in (-1,0,1):
                    if dy == 0 and dx == 0:
                        continue
                    ny, nx = y+dy, x+dx
                    if 0 <= ny < h and 0 <= nx < w:
                        if skel[ny, nx] and not comp_mask[ny, nx]:
                            boundary_neighbors.add((ny, nx))

        bn_list = list(boundary_neighbors)

        if len(bn_list) == 0:
            degree = 0
        else:
            adj = [[] for _ in bn_list]
            for i,(y,x) in enumerate(bn_list):
                for j,(yy,xx) in enumerate(bn_list):
                    if i==j: continue
                    if max(abs(y-yy), abs(x-xx)) <= 1:
                        adj[i].append(j)

            visited = [False]*len(bn_list)
            components = 0
            for i in range(len(bn_list)):
                if visited[i]: continue
                components += 1
                stack = [i]
                visited[i] = True
                while stack:
                    u = stack.pop()
                    for v in adj[u]:
                        if not visited[v]:
                            visited[v] = True
                            stack.append(v)
            degree = components

        centroid_y = int(np.mean(comp_coords[:,0]))
        centroid_x = int(np.mean(comp_coords[:,1]))

        junctions.append({
            'centroid': (centroid_y, centroid_x),
            'degree': int(degree),
            'component_pixels': comp_coords,
            'boundary_neighbors': bn_list
        })

    return junctions


def calculate_centroid(points):
    """Вычисляет центроид (среднее) для списка объектов Point."""
    if not points:
        return None 
    
    N = len(points)
    sum_x = sum(p.x for p in points)
    sum_y = sum(p.y for p in points)
    sum_z = sum(p.z for p in points)
    
    return Point(sum_x / N, sum_y / N, sum_z / N)



def stabilize_branches(detected_points):
    """Стабилизирует обнаруженные точки веток"""
    
    global branch_candidates, frame_counter 
    frame_counter += 1


    for pt in detected_points:
        matched = False
        for c in branch_candidates:
            if point_dist(pt, c["pos"]) < MERGE_DIST:
                c["history"].append(frame_counter)
                
                if c["is_frozen"]:
                    pass 
                else:
                    c["temp_hits"].append(pt) 
                    
                    if len(c["temp_hits"]) >= FREEZE_COUNT:
                        c["pos"] = calculate_centroid(c["temp_hits"])
                        c["is_frozen"] = True
                        c["temp_hits"].clear()
                    else:
                        c["pos"] = calculate_centroid(c["temp_hits"])
                        
                matched = True
                break
        
        if not matched:
            branch_candidates.append({
                "pos": Point(pt.x, pt.y, pt.z),
                "history": deque([frame_counter], maxlen=BRANCH_TIME_WINDOW),
                "temp_hits": deque([pt], maxlen=FREEZE_COUNT),
                "is_frozen": False
            })

    stable = []
    for c in branch_candidates:
        hits_in_window = [
            f for f in c["history"] 
            if f > frame_counter - BRANCH_TIME_WINDOW
        ]
        
        frequency = len(hits_in_window) / BRANCH_TIME_WINDOW
        
        if frequency >= MIN_HIT_FREQUENCY:
            stable.append(c["pos"]) 

    merged_stable = []
    for pt in stable:
        if all(point_dist(pt, m) > MERGE_DIST for m in merged_stable):
            merged_stable.append(pt)

    return merged_stable


def prune_junction_branches(skel_bool, min_length=PRUNE_MIN_LENGTH):
    skel = skel_bool.copy().astype(np.uint8)
    h, w = skel.shape

    junctions_data = find_junctions_and_degrees(skel_bool)

    junction_pixels = set()
    for j in junctions_data:
        for py, px in j['component_pixels']:
            junction_pixels.add((py, px))

    kernel = np.array([[1,1,1],[1,0,1],[1,1,1]], dtype=np.uint8)
    neigh_count = cv2.filter2D(skel, cv2.CV_8U, kernel)

    pixels_to_remove = set()
    endpoint_coords = np.transpose(np.nonzero((neigh_count == 1) & (skel == 1)))

    for (y0, x0) in endpoint_coords:
        path = [(y0, x0)]
        prev = None
        y, x = y0, x0
        length = 0
        hit_junction = False

        while True:
            if length >= min_length + 2:
                break

            nbrs = neighbors(skel, y, x, h, w)
            if prev is not None:
                nbrs = [n for n in nbrs if n != prev]

            is_junction_pixel = (y, x) in junction_pixels
            if is_junction_pixel:
                hit_junction = True
                break

            if len(nbrs) == 0:
                break

            if len(nbrs) > 1:
                hit_junction = True
                break

            ny, nx = nbrs[0]
            if (ny, nx) in path:
                break

            path.append((ny, nx))
            length += 1
            prev = (y, x)
            y, x = ny, nx

        if hit_junction and length < min_length:
            for (py, px) in path:
                pixels_to_remove.add((py, px))

    for (y, x) in pixels_to_remove:
        skel[y, x] = 0

    return skel.astype(bool)

def create_rotation_matrix(roll, pitch, yaw):
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    return Rz @ Ry @ Rx

def project_points(coords_2d, telem, R, img=None, target_z=0):

    X0, Y0, Z0 = telem.x, telem.y, telem.z
    
    marker = Marker() 
    marker.header.frame_id = "aruco_map"
    marker.header.stamp = rospy.Time.now()
    marker.points = []
    marker.colors = []
    
    marker = Marker()

    try:
        if fx is None or fx == 0:
            return marker
    except NameError:
        print("Error: fx, fy, cx, cy (camera intrinsics) are not defined.")
        return marker


    for y, x in coords_2d:
        u, v = x, y

        x_cam = (u - cx) / fx
        y_cam = (v - cy) / fy

        dir_cam = np.array([y_cam, x_cam, 1.0])
        
        dir_world = R.dot(dir_cam)
        
        if abs(dir_world[2]) < 1e-6:
            continue

        t = (target_z - Z0) / dir_world[2]
        
        Xg = X0 + dir_world[0] * t
        Yg = Y0 + dir_world[1] * t
        Zg = target_z

        marker.points.append(Point(Xg, Yg, Zg))
        marker.colors.append(ColorRGBA(1.0, 1.0, 1.0, 1.0))

    return marker

def quantize_point(pt, step=0.05):
    return (round(pt.x / step) * step,
            round(pt.y / step) * step,
            round(pt.z / step) * step)

def publish_tubes_json(global_map_junctions):
    points_list = [{"x": round(pt.x, 3),
                    "y": round(pt.y, 3),
                    "z": round(pt.z, 3)} 
                   for pt in global_map_junctions]

    json_str = json.dumps({"junctions": points_list})

    tubes_pub.publish(String(data=json_str))


def add_to_global_map(points, global_list, occ_set, step):
    with global_data_lock: 
        for point in points:
            q = quantize_point(point, step)
            if q not in occ_set:
                occ_set.add(q)
                global_list.append(point)

def publish_global_maps():
    with global_data_lock:
        # skeleton map
        m = Marker()
        m.header.frame_id = "aruco_map"
        m.header.stamp = rospy.Time.now()
        m.type = Marker.POINTS
        m.scale.x = m.scale.y = 0.03
        m.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)
        m.points = global_map_skeleton
        global_map_pub.publish(m)

        # mask map
        mm = Marker()
        mm.header.frame_id = "aruco_map"
        mm.header.stamp = rospy.Time.now()
        mm.type = Marker.POINTS
        mm.scale.x = mm.scale.y = 0.015
        mm.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
        mm.points = global_map_pipe
        global_mask_pub.publish(mm)

        # junctions
        j = Marker()
        j.header.frame_id = "aruco_map"
        j.header.stamp = rospy.Time.now()
        j.type = Marker.CUBE_LIST
        j.scale.x = j.scale.y = j.scale.z = 0.5
        j.color = ColorRGBA(1.0, 0.0, 0.0, 1)
        j.points = global_map_junctions
        global_junctions_pub.publish(j)

        publish_tubes_json(global_map_junctions)



# ----- main processing -----
@long_callback
def analyze_and_publish(img):
    global fx, fy, cx, cy, frame_counter

    if not rospy.get_param('/vision_pipeline/enabled', False):
        rospy.logdebug_throttle(5, "Vision pipeline is currently disabled.")
        publish_global_maps()
        return

    if fx is None:
        return

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_color = np.array([0, 0, 10])
    upper_color = np.array([179, 255, 100])
    mask = cv2.inRange(hsv, lower_color, upper_color)

    kernel = np.ones((2, 2), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)

    pub_mask.publish(bridge.cv2_to_imgmsg(mask, encoding="mono8"))

    skeleton_bool, _ = medial_axis(mask > 0, return_distance=True)
    skeleton_pruned_bool = prune_junction_branches(skeleton_bool, min_length=PRUNE_MIN_LENGTH)
    skeleton_pruned = skeleton_pruned_bool.astype(np.uint8) * 255

    pub_skeleton.publish(bridge.cv2_to_imgmsg(skeleton_pruned, encoding="mono8"))


    junctions_data = find_junctions_and_degrees(skeleton_pruned_bool)
    coords_branches = np.array([j['centroid'] for j in junctions_data]) if len(junctions_data) > 0 else np.zeros((0,2),dtype=int)

    img_junctions = img.copy()
    for x, y in coords_branches:
        cv2.circle(img_junctions, (y, x), 3, (0,0,255), -1)

    pub_junctions.publish(bridge.cv2_to_imgmsg(img_junctions, encoding="bgr8"))

    telem = get_telemetry(frame_id="aruco_map")
    roll, pitch, yaw = telem.roll, telem.pitch, telem.yaw
    R = create_rotation_matrix(roll, pitch, yaw)

    coords_skeleton = np.argwhere(skeleton_pruned_bool > 0)
    coords_mask = np.argwhere(mask > 0)

    if len(coords_skeleton) > 0:
        skeleton_marker = project_points(coords_skeleton, telem, R)
        add_to_global_map(skeleton_marker.points, global_map_skeleton, occ_skeleton, step=0.05)

    if len(coords_mask) > 0:
        mask_coords_sampled = coords_mask[::2]
        mask_marker = project_points(mask_coords_sampled, telem, R, img)
        add_to_global_map(mask_marker.points, global_map_pipe, occ_pipe, step=0.05)

    rospy.logdebug(f"junctions found: {len(coords_branches)}")
    if len(coords_branches) > 0:
        branches_marker = project_points(coords_branches, telem, R)
        stable_junctions = stabilize_branches(branches_marker.points)
        branches_marker.points = stable_junctions
        add_to_global_map(branches_marker.points, global_map_junctions, occ_junctions, step=0.05)

    frame_counter += 1
    if frame_counter % 2 == 0:
        publish_global_maps()

# ----- ROS callbacks -----
@long_callback
def image_callback(data):
    try:
        img = bridge.imgmsg_to_cv2(data, 'bgr8')
        analyze_and_publish(img)
        cv2.waitKey(1)
    except Exception as e:
        rospy.logerr(f"image_callback error: {e}")

def camera_info_callback(msg):
    global fx, fy, cx, cy
    fx = msg.K[0]
    fy = msg.K[4]
    cx = msg.K[2]
    cy = msg.K[5]
    rospy.loginfo_once(f"Camera intrinsics: fx={fx:.2f} fy={fy:.2f} cx={cx:.2f} cy={cy:.2f}")

rospy.Subscriber("main_camera/camera_info", CameraInfo, camera_info_callback)
rospy.Subscriber("main_camera/image_raw", Image, image_callback)

rospy.spin()
cv2.destroyAllWindows()