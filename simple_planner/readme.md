# AirSim Simple Global Planner

## Overview
This is a compact example showing a global planning pipeline for AirSim based on a precomputed map (point cloud + metadata). The repository is intended for reference.

## Dependencies
- Required for AirSim integration:
	- `msgpack-rpc-python` (install: `pip3 install msgpack-rpc-python`)
	- `airsim` (install: `pip3 install airsim`)
- Optional / planner-related:
	- `cvxpy` (for minimum-snap optimization)
	- `shapely` (for polygon operations)

## Quick run
- Offline map-only test (no AirSim required):
	- `python simple_planner/test_on_maps.py /path/to/map_file.pkl`
- Run with AirSim (start AirSim first):
	- `python simple_planner/test_on_airsim.py /path/to/map_file.pkl`

## Map file format
- The planner expects a `.pkl` map file that contains a point cloud and basic metadata (e.g. resolution, map extents). The scripts read the point cloud and metadata to build occupancy maps used by the planner.
- Recommended map: `UrbanBuildings_res5_0_voxel.pkl` (5 m resolution). There is also a 1 m resolution variant which is substantially larger.

**Download prepared map files:**
- [Point Cloud Map Files](https://entuedu-my.sharepoint.com/:f:/g/personal/bohang001_e_ntu_edu_sg/EguwpDDSw2ZOp8eRtg9gtAsBEErPJRKCGqRQXM_-WVSe3w?e=TLwLdT)

**Download corresponding AirSim environment:**
- [AirSim Environment](https://entuedu-my.sharepoint.com/:f:/g/personal/bohang001_e_ntu_edu_sg/EkYv4CZOF29HpsDnKVsIRk8BpkoIh-TUcX1TTsAFLgx00g?e=c50q91)
- use `sh /path_to_Bruishify/Blocks.sh -windowed` to run the simulation

**Download the prepared AirSim settings file:**
- [AirSim Settings](https://entuedu-my.sharepoint.com/:f:/g/personal/bohang001_e_ntu_edu_sg/Eox8-kEpa0lOkwPBB4_vHnUBVQ5CLYlyAAAyIUymKN3f0A?e=sctgKV)

- `test_on_airsim.py` expects the AirSim vehicle name to be `Drone1` (see  test_on_airsim.py line 77). Ensure your `settings.json` aligns with that name if you run the AirSim simulation.

## Components
- `map_manager` (author-written):
	- Purpose: load the point cloud from the provided map file and generate an occupancy grid at a specified flight altitude.
	- Behavior: reads the map file, projects or slices the 3D point cloud at the chosen height, and outputs a 2D occupancy grid compatible with the planner.

- `global_planner` (open-source reference integration):
	- Purpose: provide a global path planning pipeline that produces a collision-free, smooth trajectory.
	- Implementation notes: this module implements an A* (or optional JPS) grid search followed by convex corridor construction and a minimum-snap trajectory optimizer. The implementation is adapted from public open-source work and is intentionally modular so you can replace it with any other planner.



## Further work / suggestions

**Please pay attention to the frame in airsim and map file**. They have same x, reverse y and z.(See line 89 and 104 in test_on_maps.py).
Airsim use negative z, since it uses NED frame.

You can replace the global planner to fit larp, or write your trajectory planner and use a simple loop to execute the trajectory.

All motion API could be found [here](https://microsoft.github.io/AirSim/api_docs/html/), like `moveByVelocityAsync`or `moveToPositionAsync`.