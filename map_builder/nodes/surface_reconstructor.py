#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np


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
        
        # Check for scipy and sklearn
        self.has_scipy = self._check_scipy()
        self.has_sklearn = self._check_sklearn()
        
        if not self.has_scipy:
            self.get_logger().warn('scipy not available - using simplified clustering')
        if not self.has_sklearn:
            self.get_logger().warn('sklearn not available - using grid-based clustering')
        
        self.get_logger().info('Surface reconstructor node initialized')
    
    def _check_scipy(self):
        try:
            from scipy.spatial import ConvexHull
            return True
        except ImportError:
            return False
    
    def _check_sklearn(self):
        try:
            from sklearn.cluster import DBSCAN
            return True
        except ImportError:
            return False

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
        """Cluster points using simple grid-based or DBSCAN clustering"""
        try:
            if self.has_sklearn:
                # Use DBSCAN if available
                from sklearn.cluster import DBSCAN
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
            else:
                # Use simple grid-based clustering
                clusters = self._grid_based_clustering(points)
            
            self.get_logger().debug(f'Found {len(clusters)} clusters')
            return clusters
            
        except Exception as e:
            self.get_logger().error(f'Error clustering points: {str(e)}')
            return []
    
    def _grid_based_clustering(self, points):
        """Simple grid-based clustering when sklearn is not available"""
        try:
            grid_size = self.clustering_eps
            clusters = []
            
            # Create a dictionary to store grid cells
            grid_cells = {}
            
            for point in points:
                # Convert point to grid coordinates
                grid_x = int(point[0] / grid_size)
                grid_y = int(point[1] / grid_size)
                grid_z = int(point[2] / grid_size)
                grid_key = (grid_x, grid_y, grid_z)
                
                if grid_key not in grid_cells:
                    grid_cells[grid_key] = []
                grid_cells[grid_key].append(point)
            
            # Group adjacent cells into clusters
            visited = set()
            
            for grid_key, cell_points in grid_cells.items():
                if grid_key in visited or len(cell_points) < self.clustering_min_samples:
                    continue
                
                # Start new cluster
                cluster_points = []
                to_visit = [grid_key]
                
                while to_visit:
                    current_key = to_visit.pop()
                    if current_key in visited:
                        continue
                    
                    visited.add(current_key)
                    if current_key in grid_cells:
                        cluster_points.extend(grid_cells[current_key])
                        
                        # Add neighboring cells
                        x, y, z = current_key
                        for dx in [-1, 0, 1]:
                            for dy in [-1, 0, 1]:
                                for dz in [-1, 0, 1]:
                                    neighbor_key = (x+dx, y+dy, z+dz)
                                    if neighbor_key not in visited and neighbor_key in grid_cells:
                                        if len(grid_cells[neighbor_key]) >= self.clustering_min_samples:
                                            to_visit.append(neighbor_key)
                
                if len(cluster_points) >= self.min_cluster_size:
                    clusters.append(np.array(cluster_points))
            
            return clusters
            
        except Exception as e:
            self.get_logger().error(f'Error in grid-based clustering: {str(e)}')
            return []

    def generate_mesh_markers(self, clusters, header):
        """Generate mesh markers from point clusters"""
        marker_array = MarkerArray()
        
        try:
            for i, cluster in enumerate(clusters):
                if len(cluster) < 4:  # Need at least 4 points for mesh
                    continue
                
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
                
                # Generate triangles
                if self.has_scipy:
                    # Use convex hull if scipy is available
                    try:
                        from scipy.spatial import ConvexHull
                        points_2d = cluster[:, :2]
                        hull_2d = ConvexHull(points_2d)
                        hull_points_3d = cluster[hull_2d.vertices]
                        
                        # Simple triangulation - connect all vertices to first vertex
                        for j in range(1, len(hull_points_3d) - 1):
                            p1 = Point()
                            p1.x, p1.y, p1.z = float(hull_points_3d[0][0]), float(hull_points_3d[0][1]), float(hull_points_3d[0][2])
                            
                            p2 = Point()
                            p2.x, p2.y, p2.z = float(hull_points_3d[j][0]), float(hull_points_3d[j][1]), float(hull_points_3d[j][2])
                            
                            p3 = Point()
                            p3.x, p3.y, p3.z = float(hull_points_3d[j+1][0]), float(hull_points_3d[j+1][1]), float(hull_points_3d[j+1][2])
                            
                            marker.points.extend([p1, p2, p3])
                    except Exception as e:
                        self.get_logger().debug(f'Convex hull failed for cluster {i}: {str(e)}')
                        continue
                else:
                    # Simple mesh generation without scipy
                    triangles = self._simple_triangulation(cluster)
                    for triangle in triangles:
                        p1 = Point()
                        p1.x, p1.y, p1.z = float(triangle[0][0]), float(triangle[0][1]), float(triangle[0][2])
                        
                        p2 = Point()
                        p2.x, p2.y, p2.z = float(triangle[1][0]), float(triangle[1][1]), float(triangle[1][2])
                        
                        p3 = Point()
                        p3.x, p3.y, p3.z = float(triangle[2][0]), float(triangle[2][1]), float(triangle[2][2])
                        
                        marker.points.extend([p1, p2, p3])
                
                if marker.points:
                    marker_array.markers.append(marker)
                        
        except Exception as e:
            self.get_logger().error(f'Error generating mesh markers: {str(e)}')
        
        return marker_array
    
    def _simple_triangulation(self, points):
        """Simple triangulation without scipy"""
        try:
            # Find boundary points in 2D (approximate convex hull)
            points_2d = points[:, :2]
            
            # Find extreme points
            min_x_idx = np.argmin(points_2d[:, 0])
            max_x_idx = np.argmax(points_2d[:, 0])
            min_y_idx = np.argmin(points_2d[:, 1])
            max_y_idx = np.argmax(points_2d[:, 1])
            
            # Get unique boundary points
            boundary_indices = list(set([min_x_idx, max_x_idx, min_y_idx, max_y_idx]))
            
            if len(boundary_indices) < 3:
                return []
            
            boundary_points = points[boundary_indices]
            
            # Simple fan triangulation from first point
            triangles = []
            for i in range(1, len(boundary_points) - 1):
                triangle = [boundary_points[0], boundary_points[i], boundary_points[i+1]]
                triangles.append(triangle)
            
            return triangles
            
        except Exception as e:
            self.get_logger().debug(f'Simple triangulation failed: {str(e)}')
            return []

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