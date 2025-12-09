#!/usr/bin/env python3
import os
import sys
import argparse
from typing import Optional
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon

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

    # Visualization (simple)
    fig, ax = plt.subplots(figsize=(10,8))
    rows, cols = grid.shape
    ax.imshow(grid, origin='lower', cmap='gray_r', alpha=0.6, extent=[0, cols, 0, rows])

    if path is not None and len(path) > 1:
        path_coords = np.array(path)
        ax.plot(path_coords[:,0], path_coords[:,1], 'y--', alpha=0.4, linewidth=1, label='A* path')
    if path_simplified is not None and len(path_simplified) > 1:
        ax.plot(path_simplified[:,0], path_simplified[:,1], 'ro--', linewidth=1, markersize=6, label='Simplified')
    if opt_traj is not None and len(opt_traj) > 1:
        ax.plot(opt_traj[:,0], opt_traj[:,1], 'bo-', linewidth=2, markersize=6, label='Minimumsnap')

    for rect in (corridor if corridor is not None else []):
        try:
            if rect is not None and len(rect) >= 3:
                poly_patch = MplPolygon(rect, closed=True, alpha=0.25, facecolor='orange', edgecolor='red')
                ax.add_patch(poly_patch)
        except Exception:
            pass

    ax.plot(start[0], start[1], 'go', markersize=10, label='Start')
    ax.plot(goal[0], goal[1], 'r*', markersize=10, label='Goal')
    ax.set_xlim(-1, cols)
    ax.set_ylim(-1, rows)
    ax.set_aspect('equal')
    ax.legend()
    plt.title('Integrated PKL -> Occupancy -> A* -> Corridor -> Minimumsnap')
    plt.tight_layout()
    plt.savefig('test_on_maps.png')
    plt.show()

