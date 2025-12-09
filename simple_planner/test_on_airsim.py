#!/usr/bin/env python3
import os
import sys
import argparse
from typing import Optional
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon
import airsim
import numpy as np


def _ensure_project_on_sys_path():
    """
    Ensure project root is on sys.path so that we can import local planners.
    """
    this_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(this_dir)
    if project_root not in sys.path:
        sys.path.append(project_root)


_ensure_project_on_sys_path()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Global static planning using Hybrid A* on a voxel PKL map.")
    parser.add_argument("--map-pkl", type=str, 
                        default="/home/liang/Projects/OmniTraffic/isaac_sim_projects/Brushify/UrbanBuildings_res5_0_voxel.pkl",
                        help="Path to *_voxel.pkl map file.")
    parser.add_argument("--z-height", type=float, default=50.0, help="Height to slice point cloud.")
    parser.add_argument("--resolution", type=float, default=5.0, help="Override resolution (meters) for occupancy grid.")
    parser.add_argument("--start", type=str, default=None, help="Start coordinate as 'x,y' in world coords. If omitted, auto-picked.")
    parser.add_argument("--goal", type=str, default=None, help="Goal coordinate as 'x,y' in map frame (world coords). If omitted, auto-picked.")
    parser.add_argument("--max-width", type=float, default=7.0, help="Corridor half-width parameter.")
    parser.add_argument("--extend", type=float, default=8.0, help="Corridor extend parameter.")
    parser.add_argument("--solver", type=str, default="OSQP", choices=["OSQP", "ECOS", "SCS"], help="QP solver for minimum-snap.")
    parser.add_argument("--lambda-center", type=float, default=2.5, help="Weight for center attraction.")
    args = parser.parse_args()

    # core modules
    from map_manager import MapManager
    from global_planner import GlobalPlanner

    # build map and occupancy
    mm = MapManager()
    mm.load_map(args.map_pkl)
    grid, meta = mm.get_occupancy_grid(
        height_z=args.z_height,
        z_band=10.0,
        override_resolution=args.resolution
    )
    print(f"Occupancy grid created: rows={meta['rows']} cols={meta['cols']} resolution={meta['resolution']}")

    # choose start and goal
    sx = 0.0
    sy = 0.0
    start = mm.world_to_grid((sx, sy))
    if args.goal:
        gx, gy = [float(t) for t in args.goal.split(',')]
        goal = mm.world_to_grid((gx, gy))
    else:
        goal = mm.get_free_corner(opposite_of=start)
    print(f"Start (col,row) = {start}, Goal (col,row) = {goal}")

    # planning
    planner = GlobalPlanner(max_width=args.max_width, extend=args.extend, solver=args.solver, lambda_center=args.lambda_center, use_jps=True)
    path, path_simplified, corridor, opt_traj = planner.plan(
        grid=grid, start=start, goal=goal, corner_deg=30.0, corner_dilate=1
    )
    if path is None:
        print("A* failed to find a path on generated occupancy grid")
        sys.exit(1)
    print(f"A* path length: {len(path)}")
    if path_simplified is not None:
        print(f"Simplified path length: {len(path_simplified)}")

    # airsim workflow
    vehicle_name = 'Drone1'
    cruise_speed = 5.0 # m/s
    airsim_client = airsim.MultirotorClient()
    airsim_client.confirmConnection()
    airsim_client.enableApiControl(True, vehicle_name=vehicle_name)
    airsim_client.armDisarm(True, vehicle_name=vehicle_name)


    # take off to target height
    # In airsim, it is NED coordinate system, 
    # so we need to convert the height to negative value, and exchange x and y coordinates.
    init_kinematics = airsim.KinematicsState()
    init_kinematics.position = airsim.Vector3r(sx, -sy, -args.z_height)
    # it could change the initial oreintation to face the goal
    airsim_client.simSetKinematics(state=init_kinematics, ignore_collision=True, 
                                   vehicle_name=vehicle_name)
    airsim_client.takeoffAsync(vehicle_name=vehicle_name)
    airsim_client.simSetKinematics(state=init_kinematics, ignore_collision=True, 
                                   vehicle_name=vehicle_name)
    airsim_client.hoverAsync(vehicle_name=vehicle_name)

    
    # # follow path  
    # # convert opt_traj to world coordinates 
    opt_traj_world = mm.grid_to_world(opt_traj)
    # #
    # # convert opt_traj_world to airsim path
    airsim_path = [airsim.Vector3r(point[0], -point[1], -args.z_height) for point in opt_traj_world]
    airsim_client.simPlotLineStrip(airsim_path, color_rgba=[1.0, 0.0, 0.0, 1.0], thickness = 5.0, duration = -1.0, is_persistent = True)

    drivetrain = airsim.DrivetrainType.ForwardOnly
    yaw_mode = airsim.YawMode(False, 0) 
    airsim_client.moveOnPathAsync(airsim_path, cruise_speed, vehicle_name=vehicle_name, drivetrain=drivetrain, yaw_mode=yaw_mode).join()
    # # airsim_client.moveToPositionAsync(goal[0], goal[1], 50.0, 10.0, vehicle_name=vehicle_name)
    # # airsim_client.moveOnPathAsync(path, 10.0, vehicle_name=vehicle_name)
    # # airsim_client.landAsync(vehicle_name=vehicle_name)
    # # airsim_client.armDisarm(False, vehicle_name=vehicle_name)
    # # airsim_client.enableApiControl(False, vehicle_name=vehicle_name)
