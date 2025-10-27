# Project Setup Instructions

This document provides step-by-step instructions for setting up the OAK-D 3D mapping system.

## Prerequisites

1. **Hardware Setup**:
   - Raspberry Pi 4 with Ubuntu 22.04 (robot side)
   - OAK-D camera connected via USB 3.0
   - Laptop with Ubuntu 22.04 (visualization side)
   - Network connection between devices

2. **Software Requirements**:
   - ROS2 Jazzy installed on both machines
   - Python 3.10+
   - Git

## Step-by-Step Setup

### 1. Install ROS2 Jazzy

On both Raspberry Pi and laptop:

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS2 Jazzy
sudo apt update
sudo apt install ros-jazzy-desktop-full

# Install additional packages
sudo apt install python3-colcon-common-extensions
sudo apt install ros-jazzy-cv-bridge ros-jazzy-image-transport
sudo apt install ros-jazzy-tf2-ros ros-jazzy-tf2-geometry-msgs
sudo apt install ros-jazzy-robot-state-publisher ros-jazzy-rviz2
```

### 2. Setup Environment

Add to `~/.bashrc`:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
source ~/.bashrc
```

### 3. Install Python Dependencies

```bash
# Install pip if not available
sudo apt install python3-pip

# Install required packages
pip3 install depthai opencv-python numpy scipy scikit-learn matplotlib

# Optional: Install Open3D for advanced processing
pip3 install open3d
```

### 4. Setup Workspace

```bash
# Create workspace
mkdir -p ~/oakd_ws/src
cd ~/oakd_ws/src

# Copy the project files here (assuming you have them)
# If cloning from git:
# git clone <repository-url> .

# Build workspace
cd ~/oakd_ws
colcon build --symlink-install

# Source workspace
echo "source ~/oakd_ws/install/setup.bash" >> ~/.bashrc
source ~/oakd_ws/install/setup.bash
```

### 5. Test OAK-D Camera (Raspberry Pi only)

```bash
# Test DepthAI installation
python3 -c "import depthai as dai; print('DepthAI version:', dai.__version__)"

# Test camera detection
python3 -c "import depthai as dai; print('Devices:', len(dai.Device.getAllAvailableDevices()))"

# Run basic camera test
python3 -c "
import depthai as dai
import cv2

pipeline = dai.Pipeline()
cam_rgb = pipeline.create(dai.node.ColorCamera)
cam_rgb.setPreviewSize(640, 480)
cam_rgb.setInterleaved(False)

rgb_out = pipeline.create(dai.node.XLinkOut)
rgb_out.setStreamName('rgb')
cam_rgb.preview.link(rgb_out.input)

with dai.Device(pipeline) as device:
    q = device.getOutputQueue(name='rgb', maxSize=4, blocking=False)
    frame = q.get().getCvFrame()
    print(f'Camera working! Frame shape: {frame.shape}')
"
```

### 6. Network Configuration

Ensure both devices can communicate:

```bash
# On both machines, set the same ROS domain
export ROS_DOMAIN_ID=42

# Test network connectivity
ping <other_machine_ip>

# Check firewall (allow ROS2 ports)
sudo ufw allow 7400:7500/udp
```

### 7. First Test Run

#### Robot Side (Raspberry Pi):
```bash
cd ~/oakd_ws
source install/setup.bash

# Test with simulated data first
ros2 run oakd_driver oakd_publisher

# In another terminal, verify topics
ros2 topic list | grep oakd
ros2 topic echo /oakd/points --max-count 1
```

#### Laptop Side:
```bash
cd ~/oakd_ws
source install/setup.bash

# Start mapping pipeline
ros2 launch map_builder map_builder.launch.py

# In another terminal, start RViz2
rviz2 -d src/map_builder/rviz/map_builder.rviz
```

### 8. Full System Test

#### Robot Side:
```bash
# Run real camera driver
ros2 launch robot_side.launch.py
```

#### Laptop Side:
```bash
# Run visualization and mapping
ros2 launch laptop_side.launch.py
```

## Verification

### Check Topics
```bash
ros2 topic list
# Expected topics:
# /oakd/points
# /oakd/rgb/image_raw
# /oakd/depth/image_raw
# /map_builder/filtered_points
# /map_builder/occupancy_grid
# /map_builder/mesh_markers
```

### Check Nodes
```bash
ros2 node list
# Expected nodes:
# /oakd_node
# /point_cloud_processor
# /surface_reconstructor
# /map_builder_node
```

### Check Transforms
```bash
ros2 run tf2_tools view_frames
# Should show: map -> base_link -> oakd_frame
```

## Common Issues and Solutions

### 1. Camera Permission Issues
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER
# Log out and log back in
```

### 2. USB Bandwidth Issues
```bash
# Check USB tree
lsusb -t

# Ensure camera is on USB 3.0 port
# May need to reduce resolution or FPS in config
```

### 3. Network Discovery Issues
```bash
# Check ROS2 daemon
ros2 daemon stop
ros2 daemon start

# Restart network manager
sudo systemctl restart NetworkManager
```

### 4. Build Errors
```bash
# Clean build
cd ~/oakd_ws
rm -rf build install log
colcon build --symlink-install

# Check dependencies
rosdep install --from-paths src --ignore-src -r -y
```

### 5. Performance Issues
```bash
# Monitor CPU/Memory
htop

# Check topic rates
ros2 topic hz /oakd/points

# Adjust parameters in config files
# Increase voxel_size for better performance
# Reduce buffer_size to save memory
```

## Configuration Tips

### For Raspberry Pi 4:
- Use fast SD card (Class 10, A2)
- Enable swap if needed
- Consider reducing point cloud resolution
- Set CPU governor to performance:
  ```bash
  echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
  ```

### For Network Operation:
- Use 5GHz WiFi for better bandwidth
- Consider wired connection for stable operation
- Monitor network usage:
  ```bash
  iftop -i wlan0
  ```

### For Better Performance:
- Reduce point cloud density in config
- Increase voxel filter size
- Limit FPS for stable operation
- Use hardware acceleration if available

## Next Steps

After successful setup:

1. Calibrate camera parameters for your specific setup
2. Tune mapping parameters for your environment
3. Integrate with navigation stack if needed
4. Add custom features as required

## Troubleshooting Checklist

- [ ] ROS2 Jazzy installed and sourced
- [ ] Python dependencies installed
- [ ] Workspace built successfully
- [ ] Camera detected and working
- [ ] Network connectivity between machines
- [ ] Same ROS_DOMAIN_ID on both machines
- [ ] Topics publishing and receiving data
- [ ] Transforms properly configured
- [ ] RViz2 displaying data correctly