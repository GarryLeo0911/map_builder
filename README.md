# Map Builder for ROS2 Jazzy

A comprehensive ROS2 Jazzy package for building 3D maps from point cloud data. This package processes point clouds, performs surface reconstruction, and generates both 2D occupancy grids and 3D mesh representations for mapping and navigation.

## Features

- **Point Cloud Processing**: Advanced filtering, downsampling, and noise removal
- **Surface Reconstruction**: 3D mesh generation from point cloud data
- **Occupancy Grid Mapping**: 2D grid maps for navigation systems
- **Real-time Visualization**: Markers and point clouds for RViz2
- **Configurable Pipeline**: Adjustable parameters for different environments
- **Multi-sensor Support**: Compatible with various point cloud sources

## Architecture

The package consists of three main components:

1. **Point Cloud Processor**: Filters and preprocesses raw point cloud data
2. **Surface Reconstructor**: Generates 3D surfaces and mesh data
3. **Map Builder Node**: Coordinates mapping process and publishes results

## Dependencies

### System Dependencies
```bash
# ROS2 Jazzy
sudo apt install ros-jazzy-desktop-full

# Additional ROS2 packages
sudo apt install ros-jazzy-tf2-ros ros-jazzy-tf2-geometry-msgs
sudo apt install ros-jazzy-visualization-msgs ros-jazzy-nav-msgs
sudo apt install ros-jazzy-sensor-msgs ros-jazzy-geometry-msgs
```

### Python Dependencies
```bash
# Core scientific libraries
pip install numpy scipy scikit-learn

# Point cloud processing
pip install open3d

# Visualization
pip install matplotlib
```

## Installation

### 1. Clone Repository
```bash
# Create workspace
mkdir -p ~/mapping_ws
cd ~/mapping_ws

# Clone this package
git clone <repository-url> map_builder

# Build package
colcon build --packages-select map_builder

# Source workspace
source install/setup.bash
```

### 2. Verify Installation
```bash
# Check nodes are available
ros2 pkg executables map_builder

# Expected output:
# map_builder map_builder_node
# map_builder point_cloud_processor  
# map_builder surface_reconstructor
```

## Usage

### Launch Complete Mapping Pipeline
```bash
# Start all mapping components
ros2 launch map_builder map_builder.launch.py

# With custom parameters
ros2 launch map_builder map_builder.launch.py voxel_size:=0.05 max_range:=10.0
```

### Run Individual Components

#### Point Cloud Processor
```bash
ros2 run map_builder point_cloud_processor
```

#### Surface Reconstructor  
```bash
ros2 run map_builder surface_reconstructor
```

#### Main Map Builder
```bash
ros2 run map_builder map_builder_node
```

## Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/oakd/points` | `sensor_msgs/PointCloud2` | Input point cloud data |
| `/robot_pose` | `geometry_msgs/PoseStamped` | Robot pose (optional) |

## Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/map_builder/filtered_points` | `sensor_msgs/PointCloud2` | Processed point cloud |
| `/map_builder/accumulated_points` | `sensor_msgs/PointCloud2` | Historical point data |
| `/map_builder/occupancy_grid` | `nav_msgs/OccupancyGrid` | 2D navigation map |
| `/map_builder/mesh_markers` | `visualization_msgs/MarkerArray` | 3D mesh surfaces |
| `/map_builder/surface_markers` | `visualization_msgs/MarkerArray` | Surface point clusters |

## Configuration

### Main Parameters (`config/map_builder_params.yaml`)

```yaml
point_cloud_processor:
  ros__parameters:
    # Input/Output
    input_topic: "/oakd/points"
    output_topic: "/map_builder/filtered_points"
    
    # Filtering
    voxel_size: 0.02          # Voxel grid downsampling size
    max_range: 8.0            # Maximum point distance
    min_range: 0.3            # Minimum point distance
    
    # Outlier removal
    outlier_removal: true
    outlier_neighbors: 20     # Points to consider for outlier detection
    outlier_std_ratio: 2.0    # Standard deviation threshold
    
    # Buffer management
    buffer_size: 100          # Number of point clouds to keep
    buffer_memory_limit: 500  # MB memory limit

surface_reconstructor:
  ros__parameters:
    # Clustering
    clustering_enabled: true
    clustering_eps: 0.3       # DBSCAN epsilon parameter
    min_cluster_size: 100     # Minimum points per cluster
    
    # Surface fitting
    surface_fitting: true
    plane_distance_threshold: 0.05  # RANSAC distance threshold
    max_iterations: 1000      # RANSAC max iterations
    
    # Mesh generation
    mesh_enabled: true
    mesh_alpha: 0.5          # Alpha shape parameter
    mesh_simplification: 0.1  # Mesh simplification factor

map_builder_node:
  ros__parameters:
    # Occupancy grid
    grid_resolution: 0.05     # Grid cell size in meters
    grid_width: 400          # Grid width in cells
    grid_height: 400         # Grid height in cells
    
    # Map update
    update_rate: 2.0         # Hz
    decay_rate: 0.95         # Occupancy decay factor
    
    # Coordinate frames
    map_frame: "map"
    base_frame: "base_link"
    sensor_frame: "oakd_frame"
```

## Coordinate Frames

```
map
└── base_link
    └── oakd_frame (sensor)
```

## Visualization in RViz2

### Quick Setup
```bash
# Start RViz2 with pre-configured display
ros2 run rviz2 rviz2 -d rviz/map_builder.rviz
```

### Manual Setup
1. **Fixed Frame**: Set to `map`
2. **Add Displays**:
   - PointCloud2: `/map_builder/filtered_points`
   - PointCloud2: `/map_builder/accumulated_points`
   - Map: `/map_builder/occupancy_grid`
   - MarkerArray: `/map_builder/mesh_markers`
   - MarkerArray: `/map_builder/surface_markers`

## Integration Examples

### With Navigation Stack
```yaml
# nav2_params.yaml
global_costmap:
  global_costmap:
    plugins: ["static_layer", "obstacle_layer"]
    static_layer:
      plugin: "nav2_costmap_2d::StaticLayer"
      map_topic: "/map_builder/occupancy_grid"
```

### With Point Cloud Input
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class CustomInputNode(Node):
    def __init__(self):
        super().__init__('custom_input')
        self.publisher = self.create_publisher(
            PointCloud2, 
            '/oakd/points', 
            10)
        
    def publish_custom_cloud(self, cloud_data):
        # Publish your point cloud data
        self.publisher.publish(cloud_data)
```

## Performance Tuning

### For Real-time Processing
```yaml
# High-speed settings
point_cloud_processor:
  ros__parameters:
    voxel_size: 0.05        # Larger voxels = faster processing
    buffer_size: 50         # Smaller buffer = less memory
    outlier_removal: false  # Disable for speed

surface_reconstructor:
  ros__parameters:
    clustering_enabled: false  # Disable expensive clustering
    mesh_enabled: false       # Skip mesh generation
```

### For High Quality Maps
```yaml
# High-quality settings  
point_cloud_processor:
  ros__parameters:
    voxel_size: 0.01        # Fine detail
    outlier_removal: true   # Clean data
    buffer_size: 200        # More history

surface_reconstructor:
  ros__parameters:
    clustering_eps: 0.1     # Tight clusters
    plane_distance_threshold: 0.02  # Precise surfaces
    mesh_simplification: 0.05       # Detailed meshes
```

## Algorithms Used

### Point Cloud Processing
- **Voxel Grid Filtering**: Uniform downsampling
- **Statistical Outlier Removal**: Noise reduction
- **Range Filtering**: Distance-based culling

### Surface Reconstruction
- **DBSCAN Clustering**: Point grouping
- **RANSAC Plane Fitting**: Surface detection
- **Alpha Shapes**: Mesh generation
- **Mesh Simplification**: Performance optimization

### Occupancy Mapping
- **Bresenham Line Algorithm**: Ray tracing
- **Probabilistic Updates**: Bayesian occupancy
- **Memory Management**: Sliding window approach

## Troubleshooting

### No Point Cloud Input
```bash
# Check input topics
ros2 topic list | grep points
ros2 topic echo /oakd/points --max-count 1

# Verify transforms
ros2 run tf2_tools view_frames
```

### Poor Mapping Quality
- Increase point cloud density (decrease voxel_size)
- Adjust outlier removal parameters
- Check lighting conditions for input sensor
- Verify coordinate frame transformations

### Performance Issues
- Increase voxel_size for faster processing
- Reduce buffer_size to save memory
- Disable expensive features (clustering, meshing)
- Monitor CPU usage: `htop`

### Memory Problems
```bash
# Monitor memory usage
free -h
ros2 topic hz /map_builder/accumulated_points

# Reduce memory parameters
buffer_memory_limit: 200  # MB
buffer_size: 30          # Point clouds
```

## Development

### Building with Debug Info
```bash
colcon build --packages-select map_builder --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

### Running Tests
```bash
colcon test --packages-select map_builder
colcon test-result --verbose
```

### Code Structure
```
map_builder/
├── map_builder/
│   ├── __init__.py
│   ├── map_builder_node.py      # Main coordination node
│   ├── point_cloud_processor.py # Point cloud filtering
│   └── surface_reconstructor.py # 3D surface generation
├── config/
│   └── map_builder_params.yaml  # Configuration parameters
├── launch/
│   └── map_builder.launch.py    # Launch file
└── rviz/
    └── map_builder.rviz         # RViz configuration
```

## API Reference

### PointCloudProcessor
- `filter_points()`: Apply voxel grid and outlier removal
- `update_buffer()`: Manage point cloud history
- `transform_cloud()`: Handle coordinate transformations

### SurfaceReconstructor  
- `cluster_points()`: DBSCAN clustering
- `fit_surfaces()`: RANSAC plane fitting
- `generate_mesh()`: Alpha shape mesh creation

### MapBuilderNode
- `update_occupancy_grid()`: Generate navigation map
- `publish_visualizations()`: Create RViz markers
- `coordinate_processing()`: Manage pipeline timing

## Contributing

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## License

MIT License - see LICENSE file for details

## Support

- **Issues**: Report bugs and request features via GitHub Issues
- **Documentation**: Check ROS2 documentation for general concepts
- **Community**: ROS Discourse for mapping and navigation questions

## Related Projects

- [OAK-D Driver](https://github.com/your-org/oakd_driver) - Compatible camera driver
- [Nav2](https://github.com/ros-planning/navigation2) - ROS2 navigation stack
- [PCL](https://pointclouds.org/) - Point Cloud Library
- [Open3D](http://www.open3d.org/) - 3D data processing

## Changelog

### Version 1.0.0
- Initial release
- Point cloud processing pipeline
- Surface reconstruction
- Occupancy grid generation
- RViz2 visualization support