# RViz Plugin Error Resolution Guide

## The Problem
You're seeing RViz errors related to plugin loading, specifically with `rviz_default_plugins/` classes that can't be loaded.

## Root Cause
The issue is likely due to one of these factors:
1. **RViz Plugin Installation**: Missing or incompatible RViz plugin packages
2. **ROS2 Version Mismatch**: Plugin names changed between ROS2 versions
3. **Configuration Cache**: RViz is using cached configuration data

## Immediate Solutions

### Solution 1: Use Minimal Configuration (Recommended)
I've created a minimal RViz configuration that avoids problematic plugins:

```bash
# Use the new minimal configuration
ros2 launch map_builder oakd_map_builder.launch.py
```

This will use `rviz/map_builder_minimal.rviz` which only includes essential displays.

### Solution 2: Test Without RViz First
Verify the system works without RViz:

```bash
# Launch everything except RViz
ros2 launch map_builder oakd_no_rviz.launch.py

# In another terminal, check topics are working
ros2 topic list
ros2 topic echo /oak/points --no-arr

# Then start RViz separately
ros2 run rviz2 rviz2 -d install/map_builder/share/map_builder/rviz/map_builder_minimal.rviz
```

### Solution 3: Start RViz Fresh
Start RViz without any configuration and manually add displays:

```bash
# Start clean RViz
ros2 run rviz2 rviz2

# Then manually add:
# 1. Set Fixed Frame to "map"
# 2. Add PointCloud2 display -> Topic: /oak/points
# 3. Add PointCloud2 display -> Topic: /map_builder/filtered_points  
# 4. Add Map display -> Topic: /map_builder/occupancy_grid
# 5. Add TF display
```

## System Verification

### Check if OAK-D is Working
```bash
# List topics - you should see /oak/* topics
ros2 topic list | grep oak

# Check point cloud data
ros2 topic hz /oak/points
ros2 topic echo /oak/points --no-arr | head -20
```

### Check if Map Builder is Working
```bash
# Check processed data
ros2 topic hz /map_builder/filtered_points
ros2 topic hz /map_builder/occupancy_grid

# Monitor processing
ros2 node list
ros2 node info /point_cloud_processor
```

## Plugin Installation Fix

If you want to fix the plugin issue permanently:

```bash
# Install/reinstall RViz plugins
sudo apt update
sudo apt install ros-jazzy-rviz2 ros-jazzy-rviz-default-plugins

# Clear RViz cache
rm -rf ~/.rviz2/

# Rebuild workspace
cd your_workspace
colcon build --packages-select map_builder
source install/setup.bash
```

## Alternative Visualization

If RViz continues to cause issues, you can visualize data using:

### 1. Command Line Monitoring
```bash
# Monitor data flow
watch 'ros2 topic list | grep -E "(oak|map_builder)"'
ros2 topic hz /oak/points
```

### 2. RQT Tools
```bash
# Use RQT for visualization
rqt
# Then add: Plugins > Visualization > Plot, Image View, etc.
```

### 3. Web-based Visualization
Consider using foxglove-studio or other web-based tools that don't require RViz.

## Expected Behavior

When everything is working correctly, you should see:

1. **Topics Active**:
   - `/oak/points` - Raw point cloud from camera
   - `/oak/rgb/image_raw` - RGB camera feed  
   - `/oak/depth/image_raw` - Depth image
   - `/map_builder/filtered_points` - Processed point cloud
   - `/map_builder/occupancy_grid` - Generated map

2. **In RViz**:
   - Point clouds from the OAK-D camera
   - Filtered/processed point clouds
   - Occupancy grid map being built in real-time
   - Transform frames showing camera position

3. **No Errors**:
   - No plugin loading errors
   - Smooth point cloud updates
   - Map building in real-time

## Quick Test Command

To quickly verify everything is working:
```bash
# This should show data flowing
ros2 launch map_builder oakd_no_rviz.launch.py &
sleep 5
echo "=== Checking Topics ==="
ros2 topic list | grep -E "(oak|map_builder)"
echo "=== Checking Point Cloud Rate ==="
timeout 10 ros2 topic hz /oak/points
```

If you see data flowing, the integration is working and the issue is purely with RViz configuration.