#!/usr/bin/env python3
import rospy
import math
import time
from geometry_msgs.msg import Point
from clover import srv
from mavros_msgs.srv import CommandBool, SetMode, CommandLong
from std_srvs.srv import Trigger
from clover.srv import SetLEDEffect
from visualization_msgs.msg import Marker

EXPLORATION_SPEED = 0.4
EXPLORATION_HEIGHT = 1.7
HOME_POS = {'x': 0.0, 'y': 0.0, 'z': EXPLORATION_HEIGHT}
JUNCTION_RADIUS = 0.4
MAX_JUNCTIONS_TO_FIND = 5

global_junction_points = []

rospy.init_node('flight_control')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)
set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect, persistent=True)

set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
command_long = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)

def get_mission_command():
    return rospy.get_param("/mission_command", "idle")

def navigate_interruptible(x, y, z, speed=EXPLORATION_SPEED, frame_id='aruco_map', tolerance=0.2, auto_arm=False):

    rospy.loginfo(f"Navigating to ({x:.2f}, {y:.2f}, {z:.2f})...")
    res = navigate(x=x, y=y, z=z, speed=speed, frame_id=frame_id, auto_arm=auto_arm)
    if not res.success:
        rospy.logerr("Navigation rejected")
        return False

    while not rospy.is_shutdown():
        cmd = get_mission_command()
        if cmd != 'start':
            rospy.logwarn(f"Navigation interrupted by command: {cmd}")
            return False

        telem = get_telemetry(frame_id='navigate_target')
        dist = math.sqrt(telem.x**2 + telem.y**2 + telem.z**2)
        if dist < tolerance:
            return True
        rospy.sleep(0.2)
    return False

def quantize_point(pt, step=0.2):
    x = pt.x if isinstance(pt, Point) else pt['x']
    y = pt.y if isinstance(pt, Point) else pt['y']
    return (round(x / step), round(y / step))

def select_next_junction(current_pos, explored):
    min_dist = float('inf')
    next_junction = None
    for j in global_junction_points:
        q = quantize_point(j)
        if q in explored:
            continue
        dist = math.sqrt((current_pos.x - j.x)**2 + (current_pos.y - j.y)**2)
        if dist < min_dist:
            min_dist = dist
            next_junction = j
    return next_junction

def junction_callback(msg):
    global global_junction_points
    global_junction_points = msg.points

rospy.Subscriber("pipeline_global_junctions", Marker, junction_callback)

def main():
    rospy.loginfo("Flight Control Ready. Waiting for 'start' command...")
    explored = set()

    while not rospy.is_shutdown():
        cmd = get_mission_command()

        # --- KILL SWITCH ---
        if cmd == 'kill':
            rospy.logwarn("KILL SWITCH ACTIVATED - Attempting Force Disarm...")
            
            try:
                rospy.wait_for_service('/mavros/cmd/command', timeout=2.0)
                
                res = command_long(
                    broadcast=False, 
                    command=400, 
                    param1=0.0,
                    param2=21196.0,
                    param3=0, param4=0, param5=0, param6=0, param7=0
                )
                
                if res.success:
                    rospy.logwarn("DRONE FORCE DISARMED")
                else:
                    rospy.logerr("FORCE DISARM FAILED")
            
            except Exception as e:
                rospy.logerr(f"Force Disarm service error: {e}")

            rospy.set_param("/mission_command", "idle")
            return

        # --- STOP ---
        if cmd == 'stop':
            rospy.loginfo("STOP command received! Landing at current position.")
            land()
            rospy.set_param("/mission_command", "idle")
            time.sleep(1)
            continue

        # --- START ---
        if cmd == 'start':

            rospy.loginfo("Taking off...")
            set_effect(effect='rainbow')
            
            if not navigate_interruptible(HOME_POS['x'], HOME_POS['y'], HOME_POS['z'], frame_id='body', auto_arm=True):
                continue

            if not navigate_interruptible(1, 1, EXPLORATION_HEIGHT, frame_id='aruco_map'):
                continue
            
            VISION_ENABLE_PARAM = '/vision_pipeline/enabled'
            try:
                rospy.set_param(VISION_ENABLE_PARAM, True)
                time.sleep(1.5)
                rospy.loginfo(f"Vision Pipeline enabled for exploration.")
            except Exception as e:
                rospy.logerr(f"Failed to set ROS parameter {VISION_ENABLE_PARAM}: {e}")

            while get_mission_command() == 'start' and len(explored) < MAX_JUNCTIONS_TO_FIND:
                telem = get_telemetry(frame_id='aruco_map')
                current_pos = Point(telem.x, telem.y, telem.z)

                target = select_next_junction(current_pos, explored)
                if target is None:
                    rospy.loginfo_throttle(5, "Waiting for new junctions...")
                    rospy.sleep(1)
                    continue

                q = quantize_point(target)
                rospy.loginfo(f"Navigating to junction: {target.x:.2f}, {target.y:.2f}")
                
                if navigate_interruptible(target.x, target.y, EXPLORATION_HEIGHT):
                    explored.add(q)
                    set_effect(effect='blink', r=0, g=255, b=0)
                    rospy.loginfo(f"Junction explored! {len(explored)}/{MAX_JUNCTIONS_TO_FIND}")
                    rospy.sleep(1)
                else:
                    break 

            if get_mission_command() != 'start':
                continue

            VISION_ENABLE_PARAM = '/vision_pipeline/enabled'
            try:
                rospy.set_param(VISION_ENABLE_PARAM, False)
                rospy.loginfo(f"Vision Pipeline disabled.")
            except Exception as e:
                rospy.logerr(f"Failed to set ROS parameter {VISION_ENABLE_PARAM}: {e}")
            # -----------------------------------------------------------------

            rospy.loginfo("Mission complete. Returning home.")
            if not navigate_interruptible(HOME_POS['x'], HOME_POS['y'], HOME_POS['z'], speed=0.8):
                continue

            land()
            set_effect(effect='fill', r=0, g=255, b=0)
            rospy.set_param("/mission_command", "idle")

        rospy.sleep(0.2)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
