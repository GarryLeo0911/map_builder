# OAK-D Map Builder Integration

This document explains how to use the OAK-D camera with the map_builder package for real-time 3D mapping.

## Prerequisites

1. **OAK-D Driver**: Make sure you have the OAK-D driver installed:
   ```bash
   cd ~/your_workspace/src
   git clone https://github.com/GarryLeo0911/oakd_driver.git
   cd ../
   colcon build --packages-select oakd_driver
   source install/setup.bash
   ```

2. **Dependencies**: Install required dependencies:
   ```bash
   sudo apt install ros-jazzy-rviz2 ros-jazzy-rviz-default-plugins
   sudo apt install ros-jazzy-tf2-ros ros-jazzy-sensor-msgs-py
   pip install --user depthai>=2.20.0 opencv-python>=4.5.0 numpy>=1.20.0
   ```

## Quick Start

### Step 1: Test OAK-D Camera Connection
First, verify your OAK-D camera is working:
```bash
# Test camera detection
python3 scripts/device_detector.py --detect

# Launch simple OAK-D test
ros2 launch map_builder oakd_test.launch.py
```

This will start:
- OAK-D driver publishing point clouds to `/oak/points`
- RViz with basic visualization
- Transform publishers for proper coordinate frames

### Step 2: Launch Full Map Builder System
Once the camera is working, launch the complete mapping system:
```bash
ros2 launch map_builder oakd_map_builder.launch.py
```

This will start:
- OAK-D driver
- Point cloud processor (filtering and downsampling)
- Surface reconstructor
- Map builder node (occupancy grid generation)
- RViz with full mapping visualization

## Configuration Options

### Camera Settings
Adjust camera parameters in the launch command:
```bash
ros2 launch map_builder oakd_map_builder.launch.py \
    fps:=15 \
    rgb_resolution:=720p \
    depth_resolution:=720p \
    enable_ir:=false
```

### Mapping Parameters
Edit `config/oakd_map_builder_params.yaml` to adjust mapping settings:
- `voxel_size`: Point cloud downsampling resolution
- `max_range`: Maximum sensor range for mapping
- `map_resolution`: Occupancy grid resolution
- `map_width/height`: Size of the generated map

## Troubleshooting

### RViz Plugin Errors (Fixed)
The original RViz errors were caused by incorrect plugin class names. These have been fixed by updating:
- `rviz_common/` → `rviz_default_plugins/`
- Updated topic names from `/oakd/points` → `/oak/points`
- Fixed coordinate frame references

### Camera Connection Issues
If the OAK-D camera is not detected:
```bash
# Check USB permissions
sudo usermod -a -G plugdev $USER
# Log out and log back in

# Test device detection
python3 scripts/device_detector.py --detect

# For specific device
python3 scripts/device_detector.py --test --mx-id <your-device-mx-id>
```

### Performance Optimization
For better performance on resource-constrained systems:
- Reduce `fps` to 10-15
- Use `rgb_resolution:=720p` and `depth_resolution:=720p`
- Increase `voxel_size` to 0.05-0.1
- Reduce `buffer_size` in config file

## Topic Structure

The integrated system uses these key topics:
```
/oak/points              - Raw point cloud from OAK-D
/oak/rgb/image_raw       - RGB camera feed
/oak/depth/image_raw     - Depth image
/map_builder/filtered_points    - Processed point cloud
/map_builder/occupancy_grid     - Generated occupancy map
/map_builder/mesh_markers       - 3D surface mesh
```

## Coordinate Frames

The system uses this transform hierarchy:
```
map
└── base_link
    └── oak_camera_frame
        ├── oak_rgb_camera_frame
        └── oak_left_camera_optical_frame (depth frame)
```

## Advanced Usage

### Multiple Cameras
To use multiple OAK-D cameras:
```bash
# Terminal 1
ros2 launch map_builder oakd_map_builder.launch.py camera_name:=oak1 mx_id:=<device1-id>

# Terminal 2  
ros2 launch map_builder oakd_map_builder.launch.py camera_name:=oak2 mx_id:=<device2-id>
```

### Custom Pipeline
Modify the nodes in `map_builder/nodes/` to add:
- Custom point cloud filtering
- Neural network inference
- Additional sensor fusion

## Files Modified/Created

1. **Fixed RViz Configuration**: `rviz/map_builder.rviz`
2. **Updated Point Cloud Processor**: `map_builder/nodes/point_cloud_processor.py`
3. **New Integrated Launch**: `launch/oakd_map_builder.launch.py`
4. **Test Launch**: `launch/oakd_test.launch.py`
5. **Optimized Config**: `config/oakd_map_builder_params.yaml`
6. **Updated Dependencies**: `package.xml`