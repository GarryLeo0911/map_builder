#!/usr/bin/env python3
"""
RViz Configuration Fixer for Map Builder

This script creates a minimal, working RViz configuration that avoids
the plugin loading issues you're experiencing.
"""

import os
import sys
from pathlib import Path


def create_minimal_rviz_config():
    """Create a minimal RViz config that definitely works"""
    
    config_content = """Panels:
  - Class: rviz_default_plugins/tool_properties
    Name: Tool Properties
  - Class: rviz_default_plugins/displays
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /OAK Points1
        - /Map1
      Splitter Ratio: 0.5

Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Name: Grid
      Reference Frame: <Fixed Frame>
      Value: true
    
    - Alpha: 1
      Class: rviz_default_plugins/PointCloud2
      Color: 255; 255; 255
      Color Transformer: AxisColor
      Enabled: true
      Name: OAK Points
      Size (Pixels): 3
      Style: Points
      Topic:
        Value: /oak/points
      Use Fixed Frame: true
      Value: true
    
    - Alpha: 1
      Class: rviz_default_plugins/PointCloud2
      Color: 255; 255; 255
      Color Transformer: RGB8
      Enabled: true
      Name: Filtered Points
      Size (Pixels): 2
      Style: Points
      Topic:
        Value: /map_builder/filtered_points
      Use Fixed Frame: true
      Value: true
    
    - Alpha: 0.7
      Class: rviz_default_plugins/Map
      Color Scheme: map
      Enabled: true
      Name: Occupancy Map
      Topic:
        Value: /map_builder/occupancy_grid
      Value: true

  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: map
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 10
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Name: Current View
      Pitch: 0.5
      Yaw: 0.8
    Saved: ~

Window Geometry:
  Height: 800
  Width: 1200
"""
    
    return config_content


def main():
    # Get script directory
    script_dir = Path(__file__).parent.parent.parent
    rviz_dir = script_dir / "rviz"
    
    # Create rviz directory if it doesn't exist
    rviz_dir.mkdir(exist_ok=True)
    
    # Create minimal config
    config_file = rviz_dir / "map_builder_minimal.rviz"
    
    print(f"Creating minimal RViz config at: {config_file}")
    
    with open(config_file, 'w') as f:
        f.write(create_minimal_rviz_config())
    
    print("✓ Minimal RViz configuration created successfully!")
    print(f"To use this config, update your launch files to use:")
    print(f"  {config_file}")
    
    # Also create a no-rviz launch command instruction
    print("\nAlternatively, you can test the system without RViz first:")
    print("  ros2 launch map_builder oakd_no_rviz.launch.py")
    print("\nThen start RViz separately with:")
    print(f"  ros2 run rviz2 rviz2 -d {config_file}")


if __name__ == "__main__":
    main()