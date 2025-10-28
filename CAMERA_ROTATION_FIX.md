# Camera Rotation Fix for 3D Mapping System

## Problem Description
The original 3D mapping system had issues when the camera was rotated - the map would still build in the same dimension/area in RViz2, indicating that the system wasn't properly handling camera rotation and coordinate transformations.

## Root Causes Identified

1. **Missing Coordinate Transformations**: The `transformPointsToMapFrame` function was not actually transforming points from camera frame to map frame.

2. **Static Transform Assumptions**: The system used only static transforms, which don't account for camera movement and rotation.

3. **No Camera Pose Tracking**: There was no mechanism to track the camera's pose as it moves and rotates in space.

4. **Incorrect Frame Handling**: Point clouds were being processed in camera frame but treated as if they were in map frame.

## Solutions Implemented

### 1. Enhanced Point Cloud Processing (`point_cloud_processor.cpp`)

- **Added proper TF2 transformations**: The `transformToMapFrame` function now properly converts point clouds from camera frame to map frame using TF2.
- **Dynamic transform handling**: The system now waits for and uses dynamic transforms when available, falling back to static transforms if needed.
- **Frame consistency**: All processed point clouds are now consistently in map frame before being passed to the map builder.

Key changes:
```cpp
// New transformation function that properly handles coordinate frames
pcl::PointCloud<pcl::PointXYZ>::Ptr transformToMapFrame(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
    const std_msgs::msg::Header& header);
```

### 2. Visual Odometry System (`visual_odometry.hpp` and `visual_odometry.cpp`)

Created a new visual odometry node that:

- **Tracks camera motion**: Uses point cloud registration (ICP/GICP) to estimate camera movement between frames.
- **Publishes dynamic transforms**: Broadcasts the camera pose relative to the map frame in real-time.
- **Validates transformations**: Implements sanity checks to prevent large jumps or invalid transformations.
- **Handles rotation properly**: Correctly estimates and applies rotational movements of the camera.

Key features:
- Uses Generalized ICP (GICP) for better accuracy with structured point clouds
- Implements motion validation to prevent tracking failures
- Publishes pose at 20Hz for smooth tracking
- Configurable parameters for different environments

### 3. Enhanced Launch Configuration (`oakd_3d_mapping.launch.py`)

- **Added visual odometry node**: The launch file now includes the visual odometry node in the processing pipeline.
- **Updated transform tree**: Modified the TF tree to use dynamic transforms from visual odometry instead of static ones.
- **Proper frame relationships**: Set up correct parent-child relationships between map, odom, base_link, and camera frames.

### 4. Updated Configuration Parameters (`oakd_map_builder_params.yaml`)

Added visual odometry parameters:
- Correspondence distance and iteration limits for ICP
- Motion validation thresholds
- Processing frequency and accuracy settings

### 5. Map Builder Integration (`map_builder.cpp`)

- **Robot pose subscription**: Now subscribes to poses from visual odometry instead of assuming static position.
- **Dynamic transform support**: Updated to work with the new coordinate transform system.
- **Improved ray tracing**: Ray tracing now works correctly with the moving camera.

## Technical Improvements

### Coordinate Frame Handling
The system now properly maintains the following coordinate frame tree:
```
map -> odom -> base_link -> oak_camera_frame -> oak_left_camera_optical_frame
```

Where:
- `map`: Fixed world frame for the global map
- `odom`: Odometry frame (static in this implementation)
- `base_link`: Robot/camera base frame (published by visual odometry)
- `oak_camera_frame`: Camera mount frame
- `oak_left_camera_optical_frame`: Camera optical frame

### Motion Estimation
The visual odometry system uses:
1. **Point cloud preprocessing**: Filtering and downsampling for consistent registration
2. **ICP/GICP registration**: Robust point cloud alignment between consecutive frames
3. **Motion validation**: Checks for reasonable translation and rotation magnitudes
4. **Cumulative transform tracking**: Maintains global pose by accumulating frame-to-frame motions

### Error Handling
- Graceful fallback when transforms are unavailable
- Validation of transformation matrices to prevent tracking failures
- Throttled warning messages to avoid log spam
- Robust exception handling throughout the pipeline

## Expected Results

After implementing these fixes, the 3D mapping system should:

1. **Correctly handle camera rotation**: When you rotate the camera, the map should build in the correct spatial relationship to the environment.
2. **Maintain spatial consistency**: Objects should appear in their correct positions relative to each other as the camera moves.
3. **Track camera movement**: The system should properly track both translation and rotation of the camera.
4. **Build coherent maps**: The accumulated point cloud and occupancy grid should represent the actual 3D structure of the environment.

## Usage

To use the enhanced system:

1. Build the package:
   ```bash
   colcon build --packages-select map_builder
   ```

2. Source the workspace:
   ```bash
   source install/setup.bash
   ```

3. Launch the enhanced 3D mapping:
   ```bash
   ros2 launch map_builder oakd_3d_mapping.launch.py
   ```

The system will now properly handle camera rotation and movement, providing accurate 3D mapping regardless of how you move or rotate the camera.

## Configuration Notes

You can tune the visual odometry performance by adjusting parameters in `config/oakd_map_builder_params.yaml`:

- `max_correspondence_distance`: Controls ICP matching strictness
- `max_translation`/`max_rotation`: Limits for motion validation
- `fitness_score_threshold`: Quality threshold for accepting registrations
- `use_gicp`: Toggle between ICP and GICP algorithms

## Troubleshooting

If you experience issues:

1. **Check TF tree**: Use `ros2 run tf2_tools view_frames` to verify the transform tree
2. **Monitor visual odometry**: Check `/visual_odometry/pose` topic for pose updates
3. **Verify point clouds**: Ensure `/oak/points` is publishing valid data
4. **Check frame IDs**: Verify that all point clouds have correct frame_id in their headers