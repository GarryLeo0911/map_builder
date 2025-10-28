# Map Builder for ROS2 Jazzy (C++)

A high-performance ROS2 Jazzy C++ package for building 3D maps from OAK-D camera point cloud data. This package leverages the Point Cloud Library (PCL) to provide real-time point cloud processing, surface reconstruction, and occupancy grid mapping for autonomous navigation and 3D mapping applications.

## Features

- **🚀 High-Performance C++**: Native PCL integration for maximum performance
- **📷 OAK-D Integration**: Seamless compatibility with [oakd_driver](https://github.com/GarryLeo0911/oakd_driver)
- **🔍 Advanced Point Cloud Processing**: PCL-based filtering, downsampling, and outlier removal
- **🗺️ Real-time Mapping**: Live occupancy grid generation for navigation
- **🎯 3D Surface Reconstruction**: Mesh generation and clustering for detailed 3D maps
- **📊 RViz2 Visualization**: Real-time 3D visualization with markers and point clouds
- **⚙️ Configurable Pipeline**: Extensive parameter tuning for different environments
- **🔧 Professional Quality**: Industry-standard algorithms and libraries

## Architecture

The package consists of three high-performance C++ nodes:

1. **Point Cloud Processor** (`point_cloud_processor`): PCL-based filtering and preprocessing
2. **Map Builder Node** (`map_builder_node`): Real-time occupancy grid generation
3. **Surface Reconstructor** (`surface_reconstructor`): 3D mesh and surface generation

## Dependencies

### System Dependencies
```bash
# ROS2 Jazzy
sudo apt install ros-jazzy-desktop-full

# PCL and related packages
sudo apt install libpcl-dev pcl-tools
sudo apt install ros-jazzy-pcl-ros ros-jazzy-pcl-conversions

# Additional ROS2 packages
sudo apt install ros-jazzy-tf2-ros ros-jazzy-tf2-geometry-msgs
sudo apt install ros-jazzy-tf2-eigen ros-jazzy-eigen3-cmake-module
sudo apt install ros-jazzy-visualization-msgs ros-jazzy-nav-msgs
```

### Hardware Requirements
- **OAK-D Camera**: Any variant (OAK-D, OAK-D-Lite, OAK-D Pro, etc.)
- **USB 3.0**: For optimal performance
- **System Memory**: Minimum 4GB RAM recommended for real-time processing

## Installation

### 1. Install oakd_driver (Required)
```bash
# Clone and build the OAK-D driver
cd ~/your_workspace/src
git clone https://github.com/GarryLeo0911/oakd_driver.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select oakd_driver
source install/setup.bash
```

### 2. Clone and Build map_builder
```bash
# Clone this package
cd ~/your_workspace/src
git clone https://github.com/GarryLeo0911/map_builder.git

# Install dependencies
cd ..
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build package
colcon build --packages-select map_builder

# Source workspace
source install/setup.bash
```

### 3. Verify Installation
```bash
# Check nodes are available
ros2 pkg executables map_builder

# Expected output:
# map_builder map_builder_node
# map_builder point_cloud_processor  
# map_builder surface_reconstructor
```

## Usage

### Quick Start with OAK-D Camera
```bash
# Connect your OAK-D camera and run complete 3D mapping pipeline
ros2 launch map_builder oakd_3d_mapping.launch.py

# With custom FPS
ros2 launch map_builder oakd_3d_mapping.launch.py fps:=15

# Without RViz (headless)
ros2 launch map_builder oakd_3d_mapping.launch.py launch_rviz:=false
```

### Individual Node Execution

#### Point Cloud Processor
```bash
ros2 run map_builder point_cloud_processor --ros-args --params-file config/map_builder_params.yaml
```

#### Map Builder Node
```bash
ros2 run map_builder map_builder_node --ros-args --params-file config/map_builder_params.yaml
```

#### Surface Reconstructor
```bash
ros2 run map_builder surface_reconstructor --ros-args --params-file config/map_builder_params.yaml
```

## Topics Interface

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/oak/points` | `sensor_msgs::PointCloud2` | Raw point cloud from OAK-D camera |
| `/robot_pose` | `geometry_msgs::PoseStamped` | Robot pose for ray tracing (optional) |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/map_builder/filtered_points` | `sensor_msgs::PointCloud2` | Processed and filtered point cloud |
| `/map_builder/accumulated_points` | `sensor_msgs::PointCloud2` | Historical accumulated point data |
| `/map_builder/occupancy_grid` | `nav_msgs::OccupancyGrid` | 2D occupancy grid for navigation |
| `/map_builder/mesh_markers` | `visualization_msgs::MarkerArray` | 3D mesh surfaces for visualization |
| `/map_builder/surface_markers` | `visualization_msgs::MarkerArray` | Surface point clusters |

## Configuration

### Key Parameters

Edit `config/map_builder_params.yaml` or `config/oakd_map_builder_params.yaml`:

```yaml
point_cloud_processor:
  ros__parameters:
    # Filtering parameters
    voxel_size: 0.05                          # Voxel grid downsampling (meters)
    max_range: 10.0                           # Maximum point distance (meters)
    min_range: 0.3                            # Minimum point distance (meters)
    statistical_outlier_nb_neighbors: 20      # Outlier detection neighbors
    statistical_outlier_std_ratio: 2.0        # Outlier standard deviation threshold
    buffer_size: 100                          # Point cloud buffer size

map_builder_node:
  ros__parameters:
    # Map configuration
    map_resolution: 0.05                      # Grid cell size (meters)
    map_width: 400                            # Grid width (cells)
    map_height: 400                           # Grid height (cells)
    map_origin_x: -10.0                       # Map origin X (meters)
    map_origin_y: -10.0                       # Map origin Y (meters)
    
    # Obstacle detection
    min_obstacle_height: 0.1                  # Minimum obstacle height (meters)
    max_obstacle_height: 2.0                  # Maximum obstacle height (meters)
    robot_radius: 0.3                         # Robot radius for planning (meters)

surface_reconstructor:
  ros__parameters:
    # Clustering
    clustering_tolerance: 0.2                 # DBSCAN clustering tolerance
    clustering_min_cluster_size: 50           # Minimum points per cluster
    clustering_max_cluster_size: 25000        # Maximum points per cluster
    
    # Mesh generation
    mesh_resolution: 0.1                      # Mesh resolution (meters)
    convex_hull_alpha: 0.05                   # Alpha shape parameter
```

## Coordinate Frames

The system uses the following coordinate frame hierarchy:

```
map
└── base_link
    └── oak_camera_frame
        ├── oak_rgb_camera_frame
        └── oak_left_camera_frame (depth frame)
```

Static transforms are automatically published by the launch file.

## Visualization in RViz2

### Automatic Setup
The launch file automatically starts RViz2 with pre-configured displays:

```bash
ros2 launch map_builder oakd_3d_mapping.launch.py
```

### Manual RViz Setup
1. **Fixed Frame**: Set to `map`
2. **Add Displays**:
   - **PointCloud2**: `/oak/points` (raw camera data)
   - **PointCloud2**: `/map_builder/filtered_points` (processed data)
   - **PointCloud2**: `/map_builder/accumulated_points` (historical data)
   - **Map**: `/map_builder/occupancy_grid` (2D navigation map)
   - **MarkerArray**: `/map_builder/mesh_markers` (3D meshes)
   - **MarkerArray**: `/map_builder/surface_markers` (surface points)
   - **TF**: Enable to see coordinate frames

## Performance Optimization

### For Real-time Processing (High FPS)
```yaml
point_cloud_processor:
  ros__parameters:
    voxel_size: 0.08              # Larger voxels = faster processing
    buffer_size: 50               # Smaller buffer = less memory
    max_range: 8.0                # Reduced range = fewer points

map_builder_node:
  ros__parameters:
    map_resolution: 0.1           # Larger cells = faster updates
```

### For High-Quality Maps (Detailed)
```yaml
point_cloud_processor:
  ros__parameters:
    voxel_size: 0.02              # Fine detail preservation
    buffer_size: 200              # More historical data

surface_reconstructor:
  ros__parameters:
    clustering_tolerance: 0.1     # Tighter clustering
    mesh_resolution: 0.05         # Detailed mesh generation
```

## Integration Examples

### With Nav2 Navigation Stack
```yaml
# nav2_params.yaml
global_costmap:
  global_costmap:
    plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
    obstacle_layer:
      plugin: "nav2_costmap_2d::ObstacleLayer"
      observation_sources: map_builder_points
      map_builder_points:
        topic: /map_builder/filtered_points
        data_type: "PointCloud2"
        min_obstacle_height: 0.1
        max_obstacle_height: 2.0
```

### Custom C++ Integration
```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

class MapConsumer : public rclcpp::Node
{
public:
    MapConsumer() : Node("map_consumer")
    {
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map_builder/occupancy_grid", 10,
            std::bind(&MapConsumer::mapCallback, this, std::placeholders::_1));
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        // Process the occupancy grid
        RCLCPP_INFO(this->get_logger(), "Received map: %dx%d @ %.3fm resolution", 
                    msg->info.width, msg->info.height, msg->info.resolution);
    }
    
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
};
```

## Algorithms and Libraries

### Point Cloud Processing (PCL-based)
- **Voxel Grid Filter**: `pcl::VoxelGrid` for uniform downsampling
- **Statistical Outlier Removal**: `pcl::StatisticalOutlierRemoval` for noise reduction
- **PassThrough Filter**: Range-based filtering
- **KdTree Search**: Efficient nearest neighbor queries

### Surface Reconstruction
- **Euclidean Clustering**: `pcl::EuclideanClusterExtraction` for surface segmentation
- **Convex Hull**: `pcl::ConvexHull` for boundary detection
- **Alpha Shapes**: Mesh triangulation for complex surfaces

### Occupancy Mapping
- **Bresenham Line Algorithm**: Ray tracing for free space detection
- **Probabilistic Updates**: Bayesian occupancy probability updates
- **Grid-based Representation**: Efficient spatial data structure

## Troubleshooting

### No Point Cloud Data
```bash
# Check OAK-D connection
ros2 topic list | grep oak
ros2 topic echo /oak/points --max-count 1

# Verify oakd_driver is running
ros2 node list | grep oakd
```

### Build Errors
```bash
# Install missing PCL dependencies
sudo apt install libpcl-dev pcl-tools

# Clean and rebuild
rm -rf build install log
colcon build --packages-select map_builder
```

### Performance Issues
```bash
# Monitor CPU usage
htop

# Check topic frequencies
ros2 topic hz /oak/points
ros2 topic hz /map_builder/filtered_points

# Reduce processing load
# Edit config/map_builder_params.yaml:
# - Increase voxel_size
# - Reduce buffer_size
# - Increase map_resolution
```

### Memory Issues
```bash
# Monitor memory usage
free -h

# Reduce memory parameters in config:
buffer_size: 50               # Reduce point cloud buffer
map_width: 200               # Smaller occupancy grid
map_height: 200
```

## Development

### Project Structure
```
map_builder/
├── CMakeLists.txt                    # C++ build configuration
├── package.xml                       # Package dependencies
├── include/map_builder/              # C++ header files
│   ├── point_cloud_processor.hpp
│   ├── map_builder.hpp
│   └── surface_reconstructor.hpp
├── src/                              # C++ source files
│   ├── point_cloud_processor.cpp
│   ├── point_cloud_processor_node.cpp
│   ├── map_builder.cpp
│   ├── map_builder_node.cpp
│   ├── surface_reconstructor.cpp
│   └── surface_reconstructor_node.cpp
├── config/                           # Configuration files
│   ├── map_builder_params.yaml
│   └── oakd_map_builder_params.yaml
├── launch/                           # Launch files
│   ├── oakd_3d_mapping.launch.py
│   ├── oakd_map_builder.launch.py
│   └── map_builder.launch.py
├── rviz/                            # RViz configurations
│   ├── map_builder_3d.rviz
│   └── map_builder.rviz
└── test/                            # Unit tests
    └── test_map_builder.cpp
```

### Building with Debug Information
```bash
colcon build --packages-select map_builder --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

### Running Tests
```bash
colcon test --packages-select map_builder
colcon test-result --verbose
```

### Code Style
This project follows ROS2 C++ coding standards:
- **Naming**: snake_case for variables and functions, PascalCase for classes
- **Documentation**: Doxygen-style comments for public APIs
- **Error Handling**: Exception-safe code with proper RAII

## Contributing

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/amazing-feature`)
3. Ensure code follows ROS2 C++ style guidelines
4. Add tests for new functionality
5. Commit your changes (`git commit -m 'Add amazing feature'`)
6. Push to the branch (`git push origin feature/amazing-feature`)
7. Open a Pull Request

## Performance Benchmarks

### Typical Performance (Intel i7, 16GB RAM)
- **Point Cloud Processing**: 15-30 Hz with 50k points/frame
- **Occupancy Grid Updates**: 2-5 Hz for 400x400 grid
- **Memory Usage**: 200-500 MB depending on buffer settings
- **CPU Usage**: 30-60% single core utilization

### Optimization Tips
- Use **voxel_size: 0.05-0.1** for real-time applications
- Set **buffer_size: 50-100** for memory-constrained systems
- Disable surface reconstruction for maximum speed
- Use **map_resolution: 0.1** for large environments

## License

MIT License - see LICENSE file for details.

## Acknowledgments

- **Point Cloud Library (PCL)**: Core 3D processing algorithms
- **ROS2 Community**: Framework and ecosystem
- **Luxonis**: OAK-D camera hardware and DepthAI
- **oakd_driver**: Camera interface and integration

## Related Projects

- **[oakd_driver](https://github.com/GarryLeo0911/oakd_driver)**: Required OAK-D camera driver
- **[Nav2](https://github.com/ros-planning/navigation2)**: ROS2 navigation stack
- **[PCL](https://pointclouds.org/)**: Point Cloud Library
- **[ROS2](https://docs.ros.org/en/jazzy/)**: Robot Operating System 2

## Support and Issues

- **GitHub Issues**: [Report bugs and request features](https://github.com/GarryLeo0911/map_builder/issues)
- **ROS Discourse**: [Community support](https://discourse.ros.org/)
- **Documentation**: [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)

## Changelog

### Version 2.0.0 (Current - C++ Implementation)
- **Complete C++ rewrite** for maximum performance
- **PCL integration** for professional-grade point cloud processing
- **Native OAK-D support** via oakd_driver integration
- **Real-time performance** optimizations
- **Memory efficiency** improvements
- **Industry-standard algorithms** implementation

### Version 1.0.0 (Legacy - Python Implementation)
- Initial Python-based implementation
- Basic point cloud processing
- Simple surface reconstruction
- Occupancy grid generation