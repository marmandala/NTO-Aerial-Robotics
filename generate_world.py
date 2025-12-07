import os
import math
import random
import numpy as np

# --- CONFIG ---
CLOVER_SIM_DIR = os.path.expanduser('~/catkin_ws/src/clover/clover_simulation')
WORLDS_DIR = os.path.join(CLOVER_SIM_DIR, 'resources', 'worlds')
CLOVER_MODELS_DIR = os.path.join(CLOVER_SIM_DIR, 'models')

WORLD_FILENAME = "nto.world"
MODEL_NAME = "pipeline_model"
MODEL_DIR = os.path.join(CLOVER_MODELS_DIR, MODEL_NAME)

# PARAMETERS
START_POINT = np.array([1.0, 1.0, 0.0])
MAIN_RADIUS = 0.1
BRANCH_RADIUS = 0.05
BRANCH_COUNT = 5
MIN_BRANCH_DIST = 0.75

BOX_HEIGHT = 0.0005

SEGMENT_INSET = 0.03


# --- PIPELINE MODEL GENERATOR ---
def generate_pipeline_model_sdf():
    L_main = random.uniform(5.0, 7.0)

    yaw1 = math.radians(random.uniform(0, 90))

    bend_angle = math.radians(random.uniform(-20, 20))

    t_bend = random.uniform(0.3 * L_main, 0.7 * L_main)

    P0 = START_POINT
    dir1 = np.array([math.cos(yaw1), math.sin(yaw1), 0.0])
    P1 = P0 + dir1 * t_bend

    L2 = L_main - t_bend
    yaw2 = yaw1 + bend_angle
    dir2 = np.array([math.cos(yaw2), math.sin(yaw2), 0.0])
    P2 = P1 + dir2 * L2

    segments = [
        {"start": P0, "length": t_bend, "yaw": yaw1, "radius": MAIN_RADIUS},
        {"start": P1, "length": L2, "yaw": yaw2, "radius": MAIN_RADIUS}
    ]

    branches = []
    branch_points = []

    for _ in range(BRANCH_COUNT):

        while True:
            s = random.uniform(0.5, L_main - 0.5)

            if s < t_bend:
                P = P0 + dir1 * s
                dir_vec = dir1
            else:
                ds = s - t_bend
                P = P1 + dir2 * ds
                dir_vec = dir2

            if all(np.linalg.norm(P - bp) >= MIN_BRANCH_DIST for bp in branch_points):
                branch_points.append(P)
                break

        side = random.choice([-1, 1])
        branch_dir = np.array([-dir_vec[1] * side, dir_vec[0] * side, 0.0])

        L_branch = random.uniform(1.5, 2.0)

        branches.append({
            "start": P,
            "dir": branch_dir,
            "length": L_branch,
            "radius": BRANCH_RADIUS
        })

    sdf_elements = ""

    for i, seg in enumerate(segments):
        seg_start = seg["start"].copy()
        seg_length = seg["length"]

        if i == 1:
            seg_start -= dir1 * SEGMENT_INSET

        center = seg_start + np.array([
            (seg_length / 2) * math.cos(seg["yaw"]),
            (seg_length / 2) * math.sin(seg["yaw"]),
            BOX_HEIGHT / 2
        ])
        pose_str = f"{center[0]:.4f} {center[1]:.4f} {center[2]:.4f} 0 0 {seg['yaw']:.4f}"

        sdf_elements += f"""
    <link name='pipe_segment_{i+1}'>
      <pose>{pose_str}</pose>
      <visual name='visual'>
        <geometry>
          <box>
            <size>{seg_length:.4f} {2*seg['radius']:.4f} {BOX_HEIGHT:.4f}</size>
          </box>
        </geometry>
        <material>
          <ambient>0.1 0.1 0.1 1</ambient>
          <diffuse>0.2 0.2 0.2 1</diffuse>
          <specular>0.05 0.05 0.05 1</specular>
        </material>
      </visual>
    </link>
"""

    for i, br in enumerate(branches):
        center = br["start"] + br["dir"] * (br["length"] / 2) + np.array([0, 0, BOX_HEIGHT / 2])
        yaw = math.atan2(br["dir"][1], br["dir"][0])
        pose_str = f"{center[0]:.4f} {center[1]:.4f} {center[2]:.4f} 0 0 {yaw:.4f}"

        sdf_elements += f"""
    <link name='branch_{i+1}'>
      <pose>{pose_str}</pose>
      <visual name='visual'>
        <geometry>
          <box>
            <size>{br['length']:.4f} {2*br['radius']:.4f} {BOX_HEIGHT:.4f}</size>
          </box>
        </geometry>
        <material>
          <ambient>0.1 0.1 0.1 1</ambient>
          <diffuse>0.2 0.2 0.2 1</diffuse>
          <specular>0.05 0.05 0.05 1</specular>
        </material>
      </visual>
    </link>
"""

    model_sdf_content = f"""<?xml version='1.0'?>
<sdf version='1.5'>
  <model name='{MODEL_NAME}'>
    <static>true</static>
    {sdf_elements}
  </model>
</sdf>
"""

    os.makedirs(MODEL_DIR, exist_ok=True)
    model_file_path = os.path.join(MODEL_DIR, 'model.sdf')

    with open(model_file_path, 'w') as f:
        f.write(model_sdf_content)

    with open(os.path.join(MODEL_DIR, 'model.config'), 'w') as f:
        f.write(f"""<?xml version="1.0"?>
<model>
  <name>{MODEL_NAME}</name>
  <version>1.0</version>
  <sdf version="1.5">model.sdf</sdf>
</model>""")

    print(f"[OK] Pipeline model generated at: {model_file_path}")
    
    return model_file_path, branch_points


# --- GENERATE WORLD ---
def generate_nto_world(model_name):
    world_file_path = os.path.join(WORLDS_DIR, WORLD_FILENAME)
    
    world_sdf = f"""<?xml version="1.0" ?>
<sdf version="1.5">
  <world name="default">
    <include><uri>model://sun</uri></include>
    <include><uri>model://parquet_plane</uri><pose>0 0 -0.01 0 0 0</pose></include>
    <include><uri>model://aruco_cmit_txt</uri></include>
    <include><uri>model://{model_name}</uri><pose>0 0 0.005 0 0 0</pose></include>
    <scene>
      <ambient>0.8 0.8 0.8 1</ambient>
      <background>0.8 0.9 1 1</background>
      <shadows>false</shadows>
      <grid>false</grid>
      <origin_visual>false</origin_visual>
    </scene>
  </world>
</sdf>
"""

    os.makedirs(WORLDS_DIR, exist_ok=True)
    with open(world_file_path, 'w') as f:
        f.write(world_sdf)

    print(f"[OK] World generated: {world_file_path}")


# --- MAIN ---
def main():
    print("--- Generating NTO Pipeline World ---")
    
    model_file_path, branch_points = generate_pipeline_model_sdf()
    generate_nto_world(MODEL_NAME)
    
    print("--- Done ---")
    
    print("\nКоординаты врезок (Branch Points):")
    for i, p in enumerate(branch_points):
        coords = f"({p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f})"
        print(f"Branch {i+1}: {coords}")


if __name__ == '__main__':
    main()