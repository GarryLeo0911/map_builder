# Build Instructions and Dependency Fix

## Fixed Compilation Issues

I've resolved the compilation errors you encountered. The main issues were:

1. **Missing TF2 dependencies** in CMakeLists.txt and package.xml
2. **Incorrect header paths** for ROS 2 Jazzy

## Changes Made

### 1. Updated CMakeLists.txt
- Added missing `tf2` dependency
- Updated all target dependencies to include complete TF2 stack

### 2. Updated package.xml
- Added `tf2` build and exec dependencies

### 3. Fixed Header Includes
- Changed `tf2_geometry_msgs/tf2_geometry_msgs.h` → `tf2_geometry_msgs/tf2_geometry_msgs.hpp`
- Changed `tf2_eigen/tf2_eigen.h` → `tf2_eigen/tf2_eigen.hpp`

## Build Instructions

### Prerequisites
Make sure you have all required dependencies installed:

```bash
# Install core ROS 2 dependencies
sudo apt update
sudo apt install ros-jazzy-tf2-eigen ros-jazzy-tf2-geometry-msgs

# Install PCL and related packages
sudo apt install ros-jazzy-pcl-ros ros-jazzy-pcl-conversions

# Install additional mapping dependencies
sudo apt install ros-jazzy-nav-msgs ros-jazzy-visualization-msgs
```

### Building the Package

1. **Clean previous build (if any)**:
   ```bash
   cd ~/ros_ws
   rm -rf build/ install/ log/
   ```

2. **Source ROS 2**:
   ```bash
   source /opt/ros/jazzy/setup.bash
   ```

3. **Build the package**:
   ```bash
   colcon build --packages-select map_builder
   ```

4. **Source the workspace**:
   ```bash
   source install/setup.bash
   ```

## Verify Installation

Test that the nodes can be found:

```bash
# Check if executables are built
ros2 pkg executables map_builder

# Should show:
# map_builder map_builder_node
# map_builder point_cloud_processor
# map_builder surface_reconstructor
# map_builder visual_odometry
```

## Launch the Enhanced 3D Mapping

```bash
ros2 launch map_builder oakd_3d_mapping.launch.py
```

## Troubleshooting

### If you still get TF2 errors:
```bash
# Install missing TF2 packages
sudo apt install ros-jazzy-tf2-tools ros-jazzy-tf2-sensor-msgs
```

### If PCL compilation fails:
```bash
# Install PCL development packages
sudo apt install libpcl-dev
```

### Check ROS 2 installation:
```bash
# Verify ROS 2 environment
printenv | grep ROS
```

### Verify TF2 packages are available:
```bash
ros2 pkg list | grep tf2
```

## Expected Output

After successful build, you should see:
```
Starting >>> map_builder
Finished <<< map_builder [X.XXs]
Summary: 1 package finished [X.XXs]
```

## Testing the Fix

Once built successfully, launch the system and test camera rotation:

1. **Start the mapping system**:
   ```bash
   ros2 launch map_builder oakd_3d_mapping.launch.py
   ```

2. **Monitor the transforms**:
   ```bash
   # In another terminal
   ros2 run tf2_tools view_frames
   ```

3. **Check visual odometry**:
   ```bash
   # Monitor camera pose updates
   ros2 topic echo /visual_odometry/pose
   ```

4. **Verify point cloud transformation**:
   ```bash
   # Check that filtered points are in map frame
   ros2 topic echo /map_builder/filtered_points --field header.frame_id
   ```

The camera rotation issue should now be resolved with proper coordinate transformations and dynamic pose tracking!