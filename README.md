# Map Builder

This package provides RTAB-Map SLAM functionality that can subscribe to camera data from a remote OAK-D driver over ROS2 network.

## Features

- Real-time SLAM mapping
- Loop closure detection
- Map visualization
- Subscribes to remote camera topics
- Database persistence

## Usage

### On the laptop (for mapping):

```bash
# Build the package
colcon build --packages-select map_builder

# Source the workspace
source install/setup.bash

# Launch the map builder (subscribes to remote camera)
ros2 launch map_builder map_builder.launch.py robot_name:=oak
```

### Parameters

- `robot_name`: Name of the remote robot/camera to subscribe to (default: "oak")
- `use_sim_time`: Whether to use simulation time (default: "false")
- `config_file`: Path to RTAB-Map configuration file

### Subscribed Topics

- `/{robot_name}/rgb/image_raw`: RGB camera images from robot
- `/{robot_name}/rgb/camera_info`: RGB camera info from robot
- `/{robot_name}/stereo/image_raw`: Depth images from robot
- `/{robot_name}/vio/odometry`: Visual-inertial odometry from robot

### Published Topics

- `/rtabmap/mapData`: Map data
- `/rtabmap/grid_map`: Occupancy grid map
- `/rtabmap/cloud_map`: Point cloud map

## Network Setup

1. Ensure both robot and laptop are on the same network
2. Set the same ROS_DOMAIN_ID on both machines:
   ```bash
   export ROS_DOMAIN_ID=42
   ```

## Workflow

1. **On the robot**: Launch the oakd_driver to publish camera data
   ```bash
   ros2 launch oakd_driver driver.launch.py
   ```

2. **On the laptop**: Launch the map_builder to receive data and build maps
   ```bash
   ros2 launch map_builder map_builder.launch.py
   ```

3. **Check topics**: Verify the data flow between robot and laptop
   ```bash
   ros2 topic list
   ros2 topic echo /oak/rgb/image_raw --max-count 1
   ```

## Configuration

Edit `config/rtabmap_config.yaml` to modify SLAM parameters:
- Detection rates
- Feature extraction settings
- Loop closure parameters
- Memory management

## Database

Maps are automatically saved to `~/.ros/rtabmap.db`. You can visualize saved maps using:
```bash
rtabmap-databaseViewer ~/.ros/rtabmap.db
```