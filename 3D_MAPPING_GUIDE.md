# 3D Mapping with OAK-D Camera

## Why Your Map Appears 2D

The map appears 2D because you're primarily seeing the **occupancy grid**, which is a 2D representation. To see the full 3D visualization, you need:

1. **Surface reconstruction markers** - 3D mesh surfaces
2. **Point cloud displays** - Raw 3D point data
3. **Proper RViz configuration** - Showing all 3D elements

## Quick Fix for 3D Visualization

### 1. Use the 3D Launch File
```bash
ros2 launch map_builder oakd_3d_mapping.launch.py
```

This launch file includes the **surface_reconstructor** node which creates 3D mesh visualizations.

### 2. Or Update Your Current RViz
If you want to keep using the current launch, add these displays in RViz:

1. **PointCloud2** → Topic: `/oak/points` (Raw camera data)
2. **PointCloud2** → Topic: `/map_builder/filtered_points` (Processed data) 
3. **PointCloud2** → Topic: `/map_builder/accumulated_points` (Historical data)
4. **MarkerArray** → Topic: `/map_builder/mesh_markers` (3D surfaces)
5. **MarkerArray** → Topic: `/map_builder/surface_markers` (Surface points)

### 3. Check What's Running
```bash
# Check if all nodes are active
ros2 node list

# You should see:
# /oakd_node
# /point_cloud_processor  
# /surface_reconstructor  ← This creates 3D visualizations
# /map_builder_node
```

## Fixing the Dependencies Issues

From your terminal output, I see several missing dependencies. Here's how to fix them:

### 1. Install Missing Python Packages
```bash
# For full 3D reconstruction capabilities
pip install scipy scikit-learn

# Basic required packages
pip install numpy
```

### 2. Install ROS Dependencies
```bash
sudo apt update
sudo apt install ros-jazzy-rviz2 ros-jazzy-rviz-default-plugins
sudo apt install python3-scipy python3-sklearn python3-numpy
```

### 3. Rebuild Your Package
```bash
cd your_workspace
colcon build --packages-select map_builder
source install/setup.bash
```

## Understanding the 3D Pipeline

Your system has multiple visualization layers:

```
OAK-D Camera
    ↓
/oak/points (Raw 3D point cloud)
    ↓
Point Cloud Processor (Filtering)
    ↓
/map_builder/filtered_points (Cleaned 3D data)
    ↓
Surface Reconstructor (3D mesh generation)
    ↓
/map_builder/mesh_markers (3D surfaces)
/map_builder/surface_markers (3D surface points)
    ↓
Map Builder (2D occupancy grid)
    ↓
/map_builder/occupancy_grid (2D map)
```

## Testing Each Layer

### 1. Test OAK-D Camera
```bash
# Check raw camera data
ros2 topic echo /oak/points --no-arr | head -10
ros2 topic hz /oak/points
```

### 2. Test Point Cloud Processing  
```bash
# Check filtered data
ros2 topic echo /map_builder/filtered_points --no-arr | head -5
ros2 topic hz /map_builder/filtered_points
```

### 3. Test Surface Reconstruction
```bash
# Check 3D mesh data
ros2 topic list | grep mesh
ros2 topic echo /map_builder/mesh_markers --no-arr | head -5
```

### 4. Test 2D Mapping
```bash
# Check occupancy grid
ros2 topic echo /map_builder/occupancy_grid --no-arr | head -5
```

## Troubleshooting Specific Issues

### Issue: "surface_reconstructor node not running"
**Solution**: Use the 3D launch file:
```bash
ros2 launch map_builder oakd_3d_mapping.launch.py
```

### Issue: "No mesh markers appearing"
**Causes**: 
- Missing scipy/sklearn dependencies
- Not enough point cloud data
- Clustering parameters too strict

**Solutions**:
```bash
# Install dependencies
pip install scipy scikit-learn

# Or adjust parameters in config/oakd_map_builder_params.yaml:
surface_reconstructor:
  ros__parameters:
    min_cluster_size: 20        # Reduce from 50
    clustering_eps: 0.3         # Increase from 0.2
    clustering_min_samples: 5   # Reduce from 10
```

### Issue: "RViz plugin errors persist"
**Solution**: Use the clean 3D configuration:
```bash
ros2 run rviz2 rviz2 -d install/map_builder/share/map_builder/rviz/map_builder_3d.rviz
```

## Expected 3D Visualization

When working correctly, you should see:

1. **Colorful point clouds** showing the environment in 3D
2. **Green mesh surfaces** representing detected planes/surfaces  
3. **Colored surface points** clustered by height
4. **2D occupancy grid** overlaid on the ground plane
5. **TF frames** showing camera and coordinate system

## Performance Optimization

For better 3D visualization:

```yaml
# In config/oakd_map_builder_params.yaml
point_cloud_processor:
  ros__parameters:
    voxel_size: 0.02           # Higher resolution
    buffer_size: 200           # More history

surface_reconstructor:
  ros__parameters:
    mesh_resolution: 0.05      # Finer mesh
    min_cluster_size: 30       # More surfaces
```

## Quick Diagnostic

Run this to check your setup:
```bash
ros2 run map_builder diagnostics
```

This will tell you exactly what's missing and how to fix it.