# AirSim Simple Package

A ROS2 package for integrating AirSim simulation with ROS2, providing drone perception and mapping capabilities.

## Table of Contents

- [Dependencies](#dependencies)
- [Configuration](#configuration)
- [Map Files](#map-files)
- [Usage](#usage)
- [Node Architecture](#node-architecture)
- [Topics](#topics)
- [Coordinate System](#coordinate-system)
- [Troubleshooting](#troubleshooting)

## Dependencies

### AirSim Environment Setup

Follow the official AirSim tutorial to configure the AirSim environment. Since virtual Python environments are not easily integrated with ROS2, we follow ROS recommendations and use the system Python environment.

ros2 has a official guide for how to use virtual env, pkease refer to https://docs.ros.org/en/humble/How-To-Guides/Using-Python-Packages.html

I find a method in integrate_conda.md to integrate conda.

**Install AirSim Python dependencies:**

```bash
# Core AirSim dependencies
pip3 install msgpack-rpc-python
pip3 install airsim
```

**Install ROS2 packages:**

```bash
# ROS2 message packages (usually installed with ROS2)
sudo apt install ros-humble-geometry-msgs
sudo apt install ros-humble-sensor-msgs
sudo apt install ros-humble-nav-msgs
sudo apt install ros-humble-tf2-ros
sudo apt install ros-humble-rviz2
```

## Configuration

### AirSim Settings

The drone name in `config/params.yaml` should match the name defined in your AirSim settings file.

**Download the prepared AirSim settings file:**
- 📁 [AirSim Settings](https://entuedu-my.sharepoint.com/:f:/g/personal/bohang001_e_ntu_edu_sg/Eox8-kEpa0lOkwPBB4_vHnUBVQ5CLYlyAAAyIUymKN3f0A?e=sctgKV)
- Place the downloaded `settings.json` file in `/home/user/Documents/AirSim/`

### Package Parameters

Edit the configuration file at `config/params.yaml` to customize:

- **Drone name in AirSim** (must match settings.json)
- **Publishing frequencies** (pose and point cloud)
- **FOV parameters** (horizontal field of view)
- **Detection range** (sensor range in meters)
- **Map boundaries and resolution**
- **Map storage directory and file names**

## Map Files

### Map File Storage

Map files are stored in the `resource/maps/` directory:

- Place your `.npy` point cloud files in `resource/maps/`
- Configure the map file name in `config/params.yaml`
- Maps should be in **NED coordinate format** (consistent with AirSim)
- See `resource/maps/README.md` for detailed instructions

### Pre-built Maps and Environment

**Download prepared map files:**
- 🗺️ [Point Cloud Map Files](https://entuedu-my.sharepoint.com/:f:/g/personal/bohang001_e_ntu_edu_sg/EguwpDDSw2ZOp8eRtg9gtAsBEErPJRKCGqRQXM_-WVSe3w?e=TLwLdT)

**Download corresponding AirSim environment:**
- 🏗️ [AirSim Environment](https://entuedu-my.sharepoint.com/:f:/g/personal/bohang001_e_ntu_edu_sg/EkYv4CZOF29HpsDnKVsIRk8BpkoIh-TUcX1TTsAFLgx00g?e=c50q91)

## Usage

### Step 1: Build the Package

```bash
# Navigate to your ROS2 workspace
cd /path/to/your/ros2_ws

# Build with symlink install for development (recommended)
colcon build --packages-select airsim_simple_pkg --symlink-install

# Source the workspace
source install/setup.bash
```

### Step 2: Start AirSim Environment

Before launching the ROS2 node, start the AirSim simulation:

```bash
# Navigate to your downloaded AirSim environment directory
cd /path/to/airsim/environment

# Launch AirSim in windowed mode
./Blocks.sh -windowed
```

### Step 3: Launch the ROS2 Node

```bash
# Launch with RViz visualization (default)
ros2 launch airsim_simple_pkg airsim_perception.launch.py

# Launch options
ros2 launch airsim_simple_pkg airsim_perception.launch.py use_rviz:=true   # Start with RViz (default)
ros2 launch airsim_simple_pkg airsim_perception.launch.py use_rviz:=false  # Start without RViz
ros2 launch airsim_simple_pkg airsim_perception.launch.py log_level:=debug # Adjust log level

# Alternative: Run node directly
ros2 run airsim_simple_pkg airsim_perception_node
```

### Monitor the Topics

```bash
# Check odometry data
ros2 topic echo /airsim/odom

# Check point cloud data  
ros2 topic echo /airsim/local_pcl

# List all topics
ros2 topic list

# Check topic information
ros2 topic info /airsim/local_pcl
```

## Node Architecture

The `airsim_perception_node` provides the following functionalities:

### Core Components

1. **AirSim Connection**: Establishes connection to AirSim simulation server
2. **Map Management**: Utilizes AirSimMapManager for efficient point cloud data handling
3. **Pose Publishing**: Publishes drone pose as odometry at high frequency (50Hz)
4. **Point Cloud Publishing**: Publishes filtered point cloud based on FOV at lower frequency (5Hz)
5. **TF Broadcasting**: Publishes coordinate frame transformations


## Topics

| Topic | Message Type | Description | Frame |
|-------|-------------|-------------|-------|
| `/airsim/odom` | `nav_msgs/Odometry` | Drone pose and velocity | `world` (NED) |
| `/airsim/local_pcl` | `sensor_msgs/PointCloud2` | Filtered point cloud in vehicle frame | `base_link` |


## Coordinate System

This package uses the **NED (North-East-Down)** coordinate system throughout, consistent with AirSim:

### Coordinate Frame Definitions

- **X-axis**: Points North (forward)
- **Y-axis**: Points East (right)  
- **Z-axis**: Points Down

### Frame Relationships

| Frame | Description | Coordinate System |
|-------|-------------|------------------|
| `world` | Global reference frame | NED coordinates |
| `base_link` | Drone body frame | NED coordinates |

### Important Notes

- ✅ All published odometry and point cloud data use NED coordinates
- ✅ AirSimMapManager's `cloud_data` is in NED coordinates (consistent with AirSim)
- ✅ In parmas.yaml, **pointcloud_output_frame** could change the frame of output point cloud.
- ✅ Ensures compatibility with other modules expecting NED coordinates

## Troubleshooting

### Common Issues

**Issue**: `No module named 'airsim'`
- **Solution**: Ensure AirSim is installed in system Python: `pip3 install airsim`

**Issue**: `GLIBCXX` version errors
- **Solution**: This indicates library version conflicts. Try using system Python instead of conda.

**Issue**: No point cloud data
- **Solution**: 
  1. Verify map files are in `resource/maps/` directory
  2. Check map file name in `config/params.yaml`
  3. Ensure AirSim simulation is running

**Issue**: Connection failed to AirSim
- **Solution**:
  1. Verify AirSim environment is running
  2. Check IP and port in configuration
  3. Ensure settings.json is properly configured

### Development Tips

- Use `--symlink-install` for development to avoid rebuilding after parameter changes
- Monitor logs with `log_level:=debug` for detailed information
- Use RViz for real-time visualization of pose and point cloud data

## License

This project is licensed under the MIT License.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## Support

For issues and questions, please create an issue in the repository.