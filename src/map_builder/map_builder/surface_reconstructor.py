#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from scipy.spatial import ConvexHull
from sklearn.cluster import DBSCAN


class SurfaceReconstructor(Node):
    """
    Node for surface reconstruction from point cloud data.
    Creates meshes and surfaces for 3D mapping visualization.
    """
    
    def __init__(self):
        super().__init__('surface_reconstructor')
        
        # Declare parameters
        self.declare_parameter('mesh_resolution', 0.1)
        self.declare_parameter('clustering_eps', 0.2)
        self.declare_parameter('clustering_min_samples', 10)
        self.declare_parameter('min_cluster_size', 50)
        self.declare_parameter('convex_hull_simplification', 0.05)
        
        # Get parameters
        self.mesh_resolution = self.get_parameter('mesh_resolution').get_parameter_value().double_value
        self.clustering_eps = self.get_parameter('clustering_eps').get_parameter_value().double_value
        self.clustering_min_samples = self.get_parameter('clustering_min_samples').get_parameter_value().integer_value
        self.min_cluster_size = self.get_parameter('min_cluster_size').get_parameter_value().integer_value
        self.hull_simplification = self.get_parameter('convex_hull_simplification').get_parameter_value().double_value
        
        # Subscribers
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            'map_builder/accumulated_points',
            self.pointcloud_callback,
            10
        )
        
        # Publishers
        self.mesh_pub = self.create_publisher(
            MarkerArray,
            'map_builder/mesh_markers',
            10
        )
        
        self.surface_pub = self.create_publisher(
            MarkerArray,
            'map_builder/surface_markers',
            10
        )
        
        self.get_logger().info('Surface reconstructor node initialized')

    def pointcloud_callback(self, msg):
        """Process point cloud and generate surface reconstruction"""
        try:
            # Convert to numpy array
            points = self.pointcloud2_to_array(msg)
            
            if points is None or len(points) < self.min_cluster_size:
                return
            
            # Cluster points to identify surfaces
            clusters = self.cluster_points(points)
            
            # Generate mesh markers for each cluster
            mesh_markers = self.generate_mesh_markers(clusters, msg.header)
            
            # Generate surface markers
            surface_markers = self.generate_surface_markers(clusters, msg.header)
            
            # Publish markers
            if mesh_markers.markers:
                self.mesh_pub.publish(mesh_markers)
            
            if surface_markers.markers:
                self.surface_pub.publish(surface_markers)
                
        except Exception as e:
            self.get_logger().error(f'Error in surface reconstruction: {str(e)}')

    def pointcloud2_to_array(self, pointcloud_msg):
        """Convert PointCloud2 message to numpy array"""
        try:
            points_list = []
            for point in pc2.read_points(pointcloud_msg, skip_nans=True):
                points_list.append([point[0], point[1], point[2]])
            
            if points_list:
                return np.array(points_list, dtype=np.float32)
            else:
                return None
                
        except Exception as e:
            self.get_logger().error(f'Error converting PointCloud2 to array: {str(e)}')
            return None

    def cluster_points(self, points):
        """Cluster points using DBSCAN to identify surfaces"""
        try:
            # Apply DBSCAN clustering
            clustering = DBSCAN(eps=self.clustering_eps, min_samples=self.clustering_min_samples)
            cluster_labels = clustering.fit_predict(points)
            
            # Group points by cluster
            clusters = []
            unique_labels = np.unique(cluster_labels)
            
            for label in unique_labels:
                if label == -1:  # Skip noise points
                    continue
                
                cluster_points = points[cluster_labels == label]
                
                if len(cluster_points) >= self.min_cluster_size:
                    clusters.append(cluster_points)
            
            self.get_logger().debug(f'Found {len(clusters)} clusters')
            return clusters
            
        except Exception as e:
            self.get_logger().error(f'Error clustering points: {str(e)}')
            return []

    def generate_mesh_markers(self, clusters, header):
        """Generate mesh markers from point clusters"""
        marker_array = MarkerArray()
        
        try:
            for i, cluster in enumerate(clusters):
                if len(cluster) < 4:  # Need at least 4 points for convex hull
                    continue
                
                # Create convex hull for the cluster
                try:
                    # Project to 2D for convex hull (use X-Y plane)
                    points_2d = cluster[:, :2]
                    hull_2d = ConvexHull(points_2d)
                    
                    # Create mesh marker
                    marker = Marker()
                    marker.header = header
                    marker.ns = "mesh_surfaces"
                    marker.id = i
                    marker.type = Marker.TRIANGLE_LIST
                    marker.action = Marker.ADD
                    
                    # Set marker properties
                    marker.scale.x = 1.0
                    marker.scale.y = 1.0
                    marker.scale.z = 1.0
                    
                    marker.color.r = 0.0
                    marker.color.g = 0.8
                    marker.color.b = 0.0
                    marker.color.a = 0.6
                    
                    # Generate triangles from convex hull
                    hull_points_3d = cluster[hull_2d.vertices]
                    
                    # Simple triangulation - connect all vertices to first vertex
                    for j in range(1, len(hull_points_3d) - 1):
                        # Triangle: first vertex, current vertex, next vertex
                        p1 = Point()
                        p1.x, p1.y, p1.z = float(hull_points_3d[0][0]), float(hull_points_3d[0][1]), float(hull_points_3d[0][2])
                        
                        p2 = Point()
                        p2.x, p2.y, p2.z = float(hull_points_3d[j][0]), float(hull_points_3d[j][1]), float(hull_points_3d[j][2])
                        
                        p3 = Point()
                        p3.x, p3.y, p3.z = float(hull_points_3d[j+1][0]), float(hull_points_3d[j+1][1]), float(hull_points_3d[j+1][2])
                        
                        marker.points.extend([p1, p2, p3])
                    
                    if marker.points:
                        marker_array.markers.append(marker)
                        
                except Exception as e:
                    self.get_logger().debug(f'Skipping cluster {i} due to convex hull error: {str(e)}')
                    continue
                    
        except Exception as e:
            self.get_logger().error(f'Error generating mesh markers: {str(e)}')
        
        return marker_array

    def generate_surface_markers(self, clusters, header):
        """Generate surface point markers from clusters"""
        marker_array = MarkerArray()
        
        try:
            for i, cluster in enumerate(clusters):
                marker = Marker()
                marker.header = header
                marker.ns = "surface_points"
                marker.id = i
                marker.type = Marker.SPHERE_LIST
                marker.action = Marker.ADD
                
                # Set marker properties
                marker.scale.x = 0.05
                marker.scale.y = 0.05
                marker.scale.z = 0.05
                
                # Color based on cluster height
                avg_height = np.mean(cluster[:, 2])
                normalized_height = max(0.0, min(1.0, (avg_height + 2.0) / 4.0))  # Normalize -2 to 2 meters
                
                marker.color.r = normalized_height
                marker.color.g = 0.5
                marker.color.b = 1.0 - normalized_height
                marker.color.a = 0.8
                
                # Add points (subsample for performance)
                step = max(1, len(cluster) // 100)  # Limit to ~100 points per cluster
                for point in cluster[::step]:
                    p = Point()
                    p.x, p.y, p.z = float(point[0]), float(point[1]), float(point[2])
                    marker.points.append(p)
                
                if marker.points:
                    marker_array.markers.append(marker)
                    
        except Exception as e:
            self.get_logger().error(f'Error generating surface markers: {str(e)}')
        
        return marker_array


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SurfaceReconstructor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()