#!/usr/bin/env python3

# Test script to check if our entry points are importable

def test_map_builder():
    try:
        from map_builder.map_builder_node import main as mb_main
        print("✓ map_builder.map_builder_node:main - OK")
    except Exception as e:
        print(f"✗ map_builder.map_builder_node:main - ERROR: {e}")
    
    try:
        from map_builder.point_cloud_processor import main as pcp_main
        print("✓ map_builder.point_cloud_processor:main - OK")
    except Exception as e:
        print(f"✗ map_builder.point_cloud_processor:main - ERROR: {e}")
    
    try:
        from map_builder.surface_reconstructor import main as sr_main
        print("✓ map_builder.surface_reconstructor:main - OK")
    except Exception as e:
        print(f"✗ map_builder.surface_reconstructor:main - ERROR: {e}")

def test_oakd_driver():
    try:
        from oakd_driver.oakd_node import main as on_main
        print("✓ oakd_driver.oakd_node:main - OK")
    except Exception as e:
        print(f"✗ oakd_driver.oakd_node:main - ERROR: {e}")
    
    try:
        from oakd_driver.oakd_publisher import main as op_main
        print("✓ oakd_driver.oakd_publisher:main - OK")
    except Exception as e:
        print(f"✗ oakd_driver.oakd_publisher:main - ERROR: {e}")

if __name__ == "__main__":
    print("Testing entry points...")
    print("\nMap Builder:")
    test_map_builder()
    print("\nOAKD Driver:")
    test_oakd_driver()