#!/usr/bin/env python3
"""
Diagnostic script for map_builder OAK-D integration
Checks dependencies and provides troubleshooting information
"""

import sys
import subprocess

def check_dependency(module_name, package_name=None):
    """Check if a Python module is available"""
    try:
        __import__(module_name)
        print(f"✓ {module_name} is available")
        return True
    except ImportError:
        print(f"✗ {module_name} is missing")
        if package_name:
            print(f"  Install with: pip install {package_name}")
        return False

def check_ros_package(package_name):
    """Check if a ROS package is available"""
    try:
        result = subprocess.run(['ros2', 'pkg', 'list'], capture_output=True, text=True)
        if package_name in result.stdout:
            print(f"✓ ROS package {package_name} is available")
            return True
        else:
            print(f"✗ ROS package {package_name} is missing")
            return False
    except Exception:
        print(f"? Could not check ROS package {package_name}")
        return False

def main():
    print("=== Map Builder OAK-D Integration Diagnostics ===\n")
    
    print("1. Checking Python dependencies:")
    deps_ok = True
    deps_ok &= check_dependency("numpy")
    deps_ok &= check_dependency("rclpy")
    deps_ok &= check_dependency("sensor_msgs_py")
    
    print("\n2. Checking optional dependencies:")
    scipy_ok = check_dependency("scipy", "scipy")
    sklearn_ok = check_dependency("sklearn", "scikit-learn")
    
    print("\n3. Checking ROS packages:")
    ros_ok = True
    ros_ok &= check_ros_package("oakd_driver")
    ros_ok &= check_ros_package("rviz2")
    ros_ok &= check_ros_package("rviz_default_plugins")
    
    print("\n4. Checking map_builder components:")
    map_builder_ok = check_ros_package("map_builder")
    
    print("\n=== Summary ===")
    if deps_ok and ros_ok and map_builder_ok:
        print("✓ All required dependencies are available")
        print("\nTo test the system:")
        print("1. ros2 launch map_builder oakd_3d_mapping.launch.py")
        print("2. Or test without RViz: ros2 launch map_builder oakd_no_rviz.launch.py")
    else:
        print("✗ Some dependencies are missing")
        print("\nSuggested fixes:")
        
        if not scipy_ok or not sklearn_ok:
            print("- For full 3D reconstruction: pip install scipy scikit-learn")
            print("- The system will work with reduced functionality without these")
        
        if not ros_ok:
            print("- Install missing ROS packages with apt install")
        
        if not map_builder_ok:
            print("- Build the map_builder package: colcon build --packages-select map_builder")
    
    print("\n=== Troubleshooting ===")
    print("If RViz shows plugin errors:")
    print("1. Try the minimal config: ros2 launch map_builder oakd_no_rviz.launch.py")
    print("2. Start RViz separately: ros2 run rviz2 rviz2")
    print("3. Clear RViz cache: rm -rf ~/.rviz2/")
    
    print("\nIf no 3D visualization:")
    print("1. Check surface_reconstructor is running: ros2 node list | grep surface")
    print("2. Check mesh topics: ros2 topic list | grep mesh")
    print("3. Use oakd_3d_mapping.launch.py instead of oakd_map_builder.launch.py")


if __name__ == "__main__":
    main()