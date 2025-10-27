# OAK-D 3D Mapping System for ROS2 Jazzy

A comprehensive ROS2 Jazzy package for building 3D maps using the OAK-D camera from Luxonis. This system is designed to run on a robot (Raspberry Pi) with the camera, sending data to a laptop for visualization and processing.

## System Architecture

```
┌─────────────────┐    Network     ┌─────────────────┐
│   Raspberry Pi  │ ─────────────► │     Laptop      │
│   (Robot Side)  │                │ (Visualization) │
│                 │                │                 │
│ • OAK-D Driver  │                │ • Map Builder   │
│ • Point Cloud   │                │ • Surface Recon │
│ • RGB/Depth     │                │ • RViz2         │
└─────────────────┘                └─────────────────┘
```

## Features

- **Real-time Point Cloud Processing**: Capture and process point clouds from OAK-D camera
- **3D Surface Reconstruction**: Build mesh surfaces from point cloud data
- **Occupancy Grid Mapping**: Generate 2D occupancy grids for navigation
- **Distributed Processing**: Robot captures data, laptop processes and visualizes
- **RViz2 Integration**: Comprehensive visualization of mapping process
- **Configurable Parameters**: Tunable filtering, clustering, and mapping parameters

## Hardware Requirements

### Robot Side (Raspberry Pi)
- Raspberry Pi 4 (recommended) or compatible SBC
- OAK-D camera from Luxonis
- Sufficient power supply for both Pi and camera
- Network connectivity (WiFi/Ethernet)

### Laptop Side
- Ubuntu 22.04 with ROS2 Jazzy
- Sufficient computing power for point cloud processing
- GPU recommended for better performance

## Software Dependencies

### System Dependencies
```bash
# ROS2 Jazzy (install following official ROS2 documentation)
sudo apt update
sudo apt install ros-jazzy-desktop-full

# Additional ROS2 packages
sudo apt install ros-jazzy-cv-bridge ros-jazzy-image-transport
sudo apt install ros-jazzy-tf2-ros ros-jazzy-tf2-geometry-msgs
sudo apt install ros-jazzy-robot-state-publisher
sudo apt install ros-jazzy-rviz2
```

### Python Dependencies
```bash
# Install pip packages
pip install depthai opencv-python numpy scipy scikit-learn matplotlib

# Optional: Open3D for advanced point cloud processing
pip install open3d
```

## Installation

### 1. Clone and Build Workspace

```bash
# Create workspace
mkdir -p ~/oakd_ws/src
cd ~/oakd_ws/src

# Clone this repository
git clone <your-repository-url> .

# Build workspace
cd ~/oakd_ws
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

### 2. Install DepthAI (for OAK-D camera)

```bash
# Install DepthAI
python3 -m pip install depthai

# Test camera connection
python3 -c "import depthai as dai; print('DepthAI version:', dai.__version__)"
```

## Configuration

### Network Setup

For distributed operation, ensure both robot and laptop can communicate:

1. **Same Network**: Connect both devices to the same WiFi network
2. **ROS Domain ID**: Set the same ROS_DOMAIN_ID on both machines:
   ```bash
   export ROS_DOMAIN_ID=42
   ```
3. **Firewall**: Ensure ROS2 ports are open (typically UDP 7400-7500)

### Parameter Configuration

Edit configuration files in `src/*/config/` directories:

- `oakd_params.yaml`: Camera parameters (resolution, FPS, etc.)
- `map_builder_params.yaml`: Mapping parameters (resolution, filtering, etc.)

## Usage

### Option 1: Distributed Operation (Recommended)

#### Robot Side (Raspberry Pi):
```bash
cd ~/oakd_ws
source install/setup.bash
ros2 launch robot_side.launch.py
```

#### Laptop Side:
```bash
cd ~/oakd_ws
source install/setup.bash
ros2 launch laptop_side.launch.py
```

### Option 2: Single Machine Testing
```bash
cd ~/oakd_ws
source install/setup.bash

# With real camera
ros2 launch complete_system.launch.py

# With test data (no camera needed)
ros2 launch complete_system.launch.py use_test_data:=true
```

### Option 3: Individual Components

#### Start OAK-D Driver:
```bash
ros2 launch oakd_driver oakd_driver.launch.py
```

#### Start Mapping Pipeline:
```bash
ros2 launch map_builder map_builder.launch.py
```

#### Start RViz2 Visualization:
```bash
ros2 run rviz2 rviz2 -d src/map_builder/rviz/map_builder.rviz
```

## Topics and Data Flow

### Published Topics

#### OAK-D Driver (`oakd_driver`):
- `/oakd/rgb/image_raw` (sensor_msgs/Image): RGB camera feed
- `/oakd/depth/image_raw` (sensor_msgs/Image): Depth image
- `/oakd/points` (sensor_msgs/PointCloud2): Raw point cloud
- `/oakd/rgb/camera_info` (sensor_msgs/CameraInfo): Camera calibration
- `/oakd/depth/camera_info` (sensor_msgs/CameraInfo): Depth camera info

#### Map Builder (`map_builder`):
- `/map_builder/filtered_points` (sensor_msgs/PointCloud2): Filtered point cloud
- `/map_builder/accumulated_points` (sensor_msgs/PointCloud2): Accumulated points
- `/map_builder/occupancy_grid` (nav_msgs/OccupancyGrid): 2D occupancy map
- `/map_builder/mesh_markers` (visualization_msgs/MarkerArray): 3D mesh surfaces
- `/map_builder/surface_markers` (visualization_msgs/MarkerArray): Surface points

### Subscribed Topics

- `/oakd/points`: Point cloud input for mapping
- `/robot_pose`: Robot pose for mapping (optional)

## Visualization in RViz2

The system includes a pre-configured RViz2 setup that displays:

1. **Raw Point Cloud**: Direct output from OAK-D camera
2. **Filtered Points**: Processed and filtered point cloud
3. **Accumulated Points**: Historical point cloud data
4. **Occupancy Grid**: 2D navigation map
5. **3D Mesh Surfaces**: Reconstructed 3D surfaces
6. **Surface Point Clusters**: Colored point clusters

### RViz2 Controls

- **Orbit View**: Navigate around the 3D scene
- **2D Nav Goal**: Set navigation goals (if using with navigation stack)
- **Measure Tool**: Measure distances in the map
- **Point Publishing**: Click to publish points for testing

## Troubleshooting

### Common Issues

#### 1. Camera Not Detected
```bash
# Check USB connection and permissions
lsusb | grep "Movidius"

# Test with DepthAI
python3 -c "import depthai as dai; print(dai.Device.getAllAvailableDevices())"
```

#### 2. No Point Cloud Data
- Check camera topics: `ros2 topic list | grep oakd`
- Verify camera parameters in `oakd_params.yaml`
- Check lighting conditions (stereo cameras need texture)

#### 3. Network Communication Issues
```bash
# Check ROS2 discovery
ros2 node list

# Test topic communication
ros2 topic echo /oakd/points --max-count 1
```

#### 4. Performance Issues
- Reduce point cloud resolution in parameters
- Increase voxel filter size for downsampling
- Adjust buffer sizes in configuration

### Debug Commands

```bash
# Check node status
ros2 node list
ros2 node info /oakd_node

# Monitor topics
ros2 topic list
ros2 topic hz /oakd/points
ros2 topic echo /oakd/points --max-count 1

# Check transforms
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_link oakd_frame

# Monitor system resources
htop
```

## Advanced Configuration

### Custom Camera Calibration

If you have custom camera calibration data:

1. Update camera intrinsics in `oakd_node.py`
2. Modify focal length and baseline parameters
3. Adjust distortion parameters if needed

### Integration with Navigation

To use with ROS2 Navigation Stack:

1. The occupancy grid is published on `/map_builder/occupancy_grid`
2. Ensure proper coordinate frame alignment
3. Configure costmap parameters to use the generated map

### Performance Tuning

Key parameters for performance optimization:

```yaml
# In map_builder_params.yaml
point_cloud_processor:
  voxel_size: 0.05  # Increase for better performance
  max_range: 5.0    # Reduce for indoor environments
  buffer_size: 50   # Reduce for lower memory usage

surface_reconstructor:
  clustering_eps: 0.3      # Adjust clustering sensitivity
  min_cluster_size: 100    # Filter small surfaces
```

## Future Enhancements

- [ ] SLAM integration with pose estimation
- [ ] Loop closure detection
- [ ] Multi-robot mapping support
- [ ] Advanced mesh reconstruction algorithms
- [ ] Integration with navigation planning
- [ ] Real-time map streaming over network

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## License

MIT License - see LICENSE file for details

## Support

For issues and questions:
1. Check the troubleshooting section above
2. Review ROS2 Jazzy documentation
3. Check DepthAI documentation for camera-specific issues
4. Open an issue in this repository

## References

- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [DepthAI Documentation](https://docs.luxonis.com/en/latest/)
- [OAK-D Camera Specifications](https://docs.luxonis.com/projects/hardware/en/latest/pages/BW1098OAK.html)
- [Point Cloud Library (PCL)](https://pointclouds.org/)
- [Open3D Documentation](http://www.open3d.org/docs/)