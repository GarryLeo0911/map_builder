# ROS Package: map_builder
Package for Surface Reconstruction from pointclouds. This repository has been migrated to ROS2 and the package was renamed to `map_builder`.

It allows a mesh reconstruction from a PointCloud2 and Point messages. It outputs *.OBJ and *.STL files and publishes visualization markers.

### Installation guide (ROS2)

Install system dependencies (example for Debian/Ubuntu):

```bash
sudo apt-get install g++ cmake libboost-all-dev libeigen3-dev libgmp-dev libgmpxx4ldbl libmpfr-dev libpng-dev
```

Install ROS2 Jazzy and source it, then build the package with colcon:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Executing (ROS2)

Launch the service (ROS2 launch files are in `launch/`):

```bash
ros2 launch map_builder recon_service_launch.py
```

For testing with sample pointcloud files, use the test launch files under `launch/`.

### Services

This package starts the service `/mesh_from_pointclouds` which receives:
- `sensor_msgs/PointCloud2` — the point cloud to reconstruct
- `geometry_msgs/Point` — a source point used by some reconstruction steps

The service returns a `success` flag and a `path` to the generated mesh file and publishes mesh markers to visualization topics.