#include "rclcpp/rclcpp.hpp"
#include <fstream>

#include "map_builder/surface_recon.hpp"
#include "map_builder/pset.hpp"
#include "map_builder/hole_filling.hpp"
#include "map_builder/rm_artifacts.hpp"
#include "map_builder/parser.hpp"
#include "map_builder/texture.hpp"

#include "map_builder/srv/mesh_from_point_cloud2.hpp"

#include <map>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <iostream>
#include <iomanip>
#include <ctime>
#include <CGAL/Vector_3.h>

#include <CGAL/Kernel_traits.h>
#include <CGAL/boost/graph/properties.h>

#include <boost/cstdint.hpp>
#include <boost/graph/graph_traits.hpp>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <iostream>
#include <chrono>

using MeshSrv = map_builder::srv::MeshFromPointCloud2;

double gridm = 0.0;
double holemaxsize = 0.0;
std::string output_folder = "";
double sample = 0.0;
double trim = 0.0;
int texturesize = 0;
int remove_borders_iterations = 0;

// ROS2 publishers
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_normal;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_inverted;

static rclcpp::Logger LOGGER = rclcpp::get_logger("map_builder");

void publishSTLMarker(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub, const std::string & stl_filepath, const std::string & frame_id)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    rclcpp::Clock clock(RCL_ROS_TIME);
    marker.header.stamp = clock.now().to_msg();

    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker.mesh_resource = "file://" + stl_filepath;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.5;

    marker.lifetime = rclcpp::Duration(0,0).to_msg();
    marker.mesh_use_embedded_materials = true;

    if (marker_pub) marker_pub->publish(marker);
    RCLCPP_INFO_STREAM(LOGGER, "Publishing frame_id: " << frame_id << " stl file:" << stl_filepath);
}

void writeSTLfromMesh(Mesh mesh_obj, string output_stl_filepath, bool flip_normal, bool is_binary_format){
    /*
     * Binary STL format
     * https://github.com/CGAL/cgal/blob/master/Polyhedron_IO/include/CGAL/IO/STL_writer.h
     */

    ofstream out(output_stl_filepath);

    if(is_binary_format) {
        // is binary
        string header = "FileType: Binary                                                                ";
        out.write(header.c_str(), 80);

        const boost::uint32_t N32 = static_cast<boost::uint32_t>(faces(mesh_obj).size());
        out.write(reinterpret_cast<const char *>(&N32), sizeof(N32));

        BOOST_FOREACH(Mesh::Face_index
                              face_index, mesh_obj.faces()) {
                        Mesh::Halfedge_index he = mesh_obj.halfedge(face_index);

                        // Get the x y z position of the 3 verfices of every face
                        vertex_descriptor v0 = mesh_obj.target(he);
                        vertex_descriptor v1 = mesh_obj.target(mesh_obj.next(he));
                        vertex_descriptor v2 = mesh_obj.source(he);

                        CPoint3 p = mesh_obj.point(v0);
                        CPoint3 q = mesh_obj.point(v1);
                        CPoint3 r = mesh_obj.point(v2);

                        if (flip_normal) {
                            q = mesh_obj.point(v0);
                            p = mesh_obj.point(v1);
                        }

                        CGAL::Vector_3<CGAL::Simple_cartesian<double>> n = CGAL::Vector_3<CGAL::Simple_cartesian<double>>(
                                1, 0, 0);
                        if (!CGAL::collinear(p, q, r)) {
                            n = CGAL::unit_normal(p, q, r);
                        }

                        const float coords[12] = {
                                static_cast<float>(n.x()), static_cast<float>(n.y()), static_cast<float>(n.z()),
                                static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z()),
                                static_cast<float>(q.x()), static_cast<float>(q.y()), static_cast<float>(q.z()),
                                static_cast<float>(r.x()), static_cast<float>(r.y()), static_cast<float>(r.z())};

                        for (int i = 0; i < 12; ++i)
                            out.write(reinterpret_cast<const char *>(&coords[i]), sizeof(coords[i]));

                        out << "  ";
                    }
    }
    else{
        // is ASCII
        out << "solid STL generated by MeshLab\n";
        BOOST_FOREACH(Mesh::Face_index
                              face_index, mesh_obj.faces()) {
                        Mesh::Halfedge_index he = mesh_obj.halfedge(face_index);

                        // Get the x y z position of the 3 verfices of every face
                        vertex_descriptor v0 = mesh_obj.target(he);
                        vertex_descriptor v1 = mesh_obj.target(mesh_obj.next(he));
                        vertex_descriptor v2 = mesh_obj.source(he);

                        CPoint3 p = mesh_obj.point(v0);
                        CPoint3 q = mesh_obj.point(v1);
                        CPoint3 r = mesh_obj.point(v2);

                        if (flip_normal) {
                            q = mesh_obj.point(v0);
                            p = mesh_obj.point(v1);
                        }

                        CGAL::Vector_3<CGAL::Simple_cartesian<double>> n = CGAL::Vector_3<CGAL::Simple_cartesian<double>>(
                                1, 0, 0);
                        if(!CGAL::collinear(p, q, r)){
                            n = CGAL::unit_normal(p, q, r);
                        }

                        out << "facet normal " << n << "\nouter loop\n";
                        out << "vertex " << p << "\n";
                        out << "vertex " << q << "\n";
                        out << "vertex " << r << "\n";
                        out << "endloop\nendfacet\n";
                    }
        out << "endsolid vcg\n";
    }
}

std::string generate_mesh(const sensor_msgs::msg::PointCloud2 & msg, const geometry_msgs::msg::Point & src, std::string output) {
    std::vector<Point> points;
    std::vector<Vector> normals;
    std::vector<Color> colors;
    std::list<PointVectorPair> estimated_pwn;
    std::vector<Point_3> orig_points;
    std::vector<unsigned int> indices;
    auto src_point = Point_3(src.x, src.y, src.z);

    FT average_spacing = 0.;

    auto start_total_time = chrono::high_resolution_clock::now();

    auto start = chrono::high_resolution_clock::now();
    Pset pointset(points, normals, colors, sample); // init my point cloud
    pointset.read_pointCloud2(msg);

    for (size_t i = 0; i < points.size(); i++) //initialize structure for Kdtree
    {
        orig_points.push_back(Point_3(points[i][0], points[i][1], points[i][2]));
        indices.push_back(i);
    }
    auto stop = chrono::high_resolution_clock::now();

    RCLCPP_INFO_STREAM(LOGGER, "Time initialize pointcloud: "
                            << float(chrono::duration_cast<chrono::microseconds>(stop - start).count() / 1000000.0)
                            << " seconds");

    // Initialize a spatial kd-tree for further search on original set of points
    start = chrono::high_resolution_clock::now();
    Tree tree(
            boost::make_zip_iterator(boost::make_tuple(orig_points.begin(), indices.begin())),
            boost::make_zip_iterator(boost::make_tuple(orig_points.end(), indices.end()))
    );

    pointset.write_ply(output + "_full_pcd_rgb.ply"); //write full rgb .ply
    stop = chrono::high_resolution_clock::now();
    RCLCPP_INFO_STREAM(LOGGER, "Time to generate full rgb *.ply file: "
                            << float(chrono::duration_cast<chrono::microseconds>(stop - start).count() / 1000000.0)
                            << " seconds");

    start = chrono::high_resolution_clock::now();
    average_spacing = CGAL::compute_average_spacing<CGAL::Sequential_tag>(orig_points.begin(), orig_points.end(), 6);
    stop = chrono::high_resolution_clock::now();
    RCLCPP_INFO_STREAM(LOGGER, "Time to compute_average_spacing: "
                            << float(chrono::duration_cast<chrono::microseconds>(stop - start).count() / 1000000.0)
                            << " seconds");

    double as_ratio = gridm * log(points.size() / 10e4);

    if (as_ratio < 0.2)
        as_ratio = 0.2;

    double cell_size = average_spacing * as_ratio;

    RCLCPP_INFO_STREAM(LOGGER, "Estimated weight for grid size w.r.t. avg. point distance: " << as_ratio);
    RCLCPP_INFO_STREAM(LOGGER, "cell_size: " << cell_size);

    start = chrono::high_resolution_clock::now();
    pointset.sample_points_cgal(cell_size, 1);
    stop = chrono::high_resolution_clock::now();
    RCLCPP_INFO_STREAM(LOGGER, "Time to sample_points_cgal: "
                            << float(chrono::duration_cast<chrono::microseconds>(stop - start).count() / 1000000.0)
                            << " seconds");

    RCLCPP_INFO_STREAM(
        LOGGER, "Sampled point set size: " << points.size() << " | Normals: " << normals.size() << " | Original ply size: "
                       << colors.size());

    start = chrono::high_resolution_clock::now();
    if (normals.size() == 0){
        //ROS_INFO_STREAM("Estimating normals");
        estimate_normals(points, src_point, estimated_pwn);
    }
    else{
        //ROS_INFO_STREAM("Registering normals normals");
        estimated_pwn = register_normals(points, grab_normals(orig_points, normals));
    }

    stop = chrono::high_resolution_clock::now();
    RCLCPP_INFO_STREAM(LOGGER, "Time to estimate normals: "
                            << float(chrono::duration_cast<chrono::microseconds>(stop - start).count() / 1000000.0)
                            << " seconds");

    RCLCPP_INFO_STREAM(LOGGER, "Points sizes:" << points.size() << " Estimated PWN size:" << estimated_pwn.size());

    start = chrono::high_resolution_clock::now();
    write_ply_wnormals(output + "_simplified_pcd_normals.ply", estimated_pwn, tree, colors);
    stop = chrono::high_resolution_clock::now();
    RCLCPP_INFO_STREAM(LOGGER, "Time to write point cloud .ply file: "
                            << float(chrono::duration_cast<chrono::microseconds>(stop - start).count() / 1000000.0)
                            << " seconds");

    start = chrono::high_resolution_clock::now();
    FT sm_angle = 20.0;     // Min triangle angle in degrees. // 20.0
    FT sm_radius = 8.0;     // Max triangle size w.r.t. point set average spacing. // 8.0
    FT sm_distance = 0.5;   // Surface Approximation error w.r.t. point set average spacing. // 0.5
    trim_mesh(reconstruct_surface(estimated_pwn, output, sm_angle, sm_radius, sm_distance), tree, trim * (double) average_spacing, output);
    stop = chrono::high_resolution_clock::now();
    RCLCPP_INFO_STREAM(LOGGER, "Time trim mesh: "
                            << float(chrono::duration_cast<chrono::microseconds>(stop - start).count() / 1000000.0)
                            << " seconds");

    start = chrono::high_resolution_clock::now();
    remove_borders(output + "temp2.off", remove_borders_iterations, output);
    stop = chrono::high_resolution_clock::now();
    RCLCPP_INFO_STREAM(LOGGER, "Time remove_bordersh: "
                            << float(chrono::duration_cast<chrono::microseconds>(stop - start).count() / 1000000.0)
                            << " seconds");

    // TESTING 11/MAI/21 
    start = chrono::high_resolution_clock::now();
    fill_hole(output + "temp3.off", holemaxsize * (double) average_spacing, output);
    stop = chrono::high_resolution_clock::now();
    RCLCPP_INFO_STREAM(LOGGER, "Time fill holes: "
                            << float(chrono::duration_cast<chrono::microseconds>(stop - start).count() / 1000000.0)
                            << " seconds");

//    start = chrono::high_resolution_clock::now();
//    texture t(tree, colors, output + "_textured.obj", texturesize);
//    t.build_png();
//    t.save_obj();
//    stop = chrono::high_resolution_clock::now();
//    ROS_INFO_STREAM("Time generate *.obj file: "
//                            << float(chrono::duration_cast<chrono::microseconds>(stop - start).count() / 1000000.0)
//                            << " seconds");

//    ROS_INFO_STREAM("Writing STL");

    // TESTING 11/MAI/21 
    // it normally is temp3.off
    std::ifstream input_off3(output + "temp3.off"); // normal is "temp3.off" change after testing
    string output_stl_filepath = output + "_mesh.stl";
    string output_stl_filepath_inverted = output + "_inverted_mesh.stl";
    Mesh mesh_obj;


    stop = chrono::high_resolution_clock::now();
    RCLCPP_INFO_STREAM(LOGGER, "Time total reconstruction: "
                            << float(chrono::duration_cast<chrono::microseconds>(stop - start_total_time).count() / 1000000.0)
                            << " seconds");

    if (!input_off3 || !(input_off3 >> mesh_obj)) {
        RCLCPP_ERROR_STREAM(LOGGER, "Not a valid off file (generate_mesh): " << output_stl_filepath);
        return "";
    } else {
        start = chrono::high_resolution_clock::now();

        writeSTLfromMesh(mesh_obj, output_stl_filepath, false, true);
        publishSTLMarker(marker_pub_normal, output_stl_filepath, msg.header.frame_id);

        writeSTLfromMesh(mesh_obj, output_stl_filepath_inverted, true, true);
        publishSTLMarker(marker_pub_inverted, output_stl_filepath_inverted, msg.header.frame_id);

        RCLCPP_INFO_STREAM(LOGGER, "STL files written to:" << output_stl_filepath << " and " << output_stl_filepath_inverted);
        stop = chrono::high_resolution_clock::now();
        RCLCPP_INFO_STREAM(LOGGER, "Time generate *.stl file: "
                                << float(chrono::duration_cast<chrono::microseconds>(stop - start).count() / 1000000.0)
                                << " seconds");
    }

    return output_stl_filepath;
}

// dynamic reconfigure is not used in ROS2 migration; parameters are declared on node and can be set at runtime

void genMeshFromPointCloudCallback(const std::shared_ptr<MeshSrv::Request> req, std::shared_ptr<MeshSrv::Response> res)
{
    RCLCPP_INFO(LOGGER, "genMeshFromPointCloudCallback called");

    try {
        const sensor_msgs::msg::PointCloud2 ptcloud = req->input;
        const geometry_msgs::msg::Point src = req->src;

        time_t rawtime;
        struct tm *timeinfo;
        char buffer[80];

        time(&rawtime);
        timeinfo = localtime(&rawtime);

        strftime(buffer, sizeof(buffer), "%d-%m-%Y_%H:%M:%S", timeinfo);

        std::string output_file = output_folder;
        char endch = output_file.back();
        if ('/' != endch && '\\' != endch) {
            output_file += "/";
        }
        output_file += buffer;

        RCLCPP_INFO_STREAM(LOGGER, "Output file:" << output_file);

        std::string output_path = generate_mesh(ptcloud, src, output_file);

        res->success = true;
        res->path = output_path;
    } catch (std::runtime_error &e) {
        RCLCPP_ERROR(LOGGER, "genMeshFromPointCloudCallback exception: %s", e.what());
        res->success = false;
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("map_builder_pointcloud_service_node");

    RCLCPP_INFO(node->get_logger(), "Start map_builder_service node");

    std::string version(std::to_string(CGAL_VERSION_NR));
    RCLCPP_INFO_STREAM(node->get_logger(), "Using CGAL lib version " << version.substr(1, 2) << "." << version.substr(3, 2));

    // declare parameters with current global defaults
    node->declare_parameter<double>("gridm", gridm);
    node->declare_parameter<double>("holemaxsize", holemaxsize);
    node->declare_parameter<std::string>("output_folder", output_folder);
    node->declare_parameter<double>("sample", sample);
    node->declare_parameter<double>("trim", trim);
    node->declare_parameter<int>("texturesize", texturesize);
    node->declare_parameter<int>("remove_borders_iterations", remove_borders_iterations);

    // create publishers
    marker_pub_normal = node->create_publisher<visualization_msgs::msg::Marker>("/reconstructed_mesh_marker_normal", 1);
    marker_pub_inverted = node->create_publisher<visualization_msgs::msg::Marker>("/reconstructed_mesh_marker_inverted", 1);

    // service
    auto service = node->create_service<MeshSrv>("/mesh_from_pointclouds", &genMeshFromPointCloudCallback);

    RCLCPP_INFO(node->get_logger(), "Spinning map_builder_service node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    RCLCPP_INFO(node->get_logger(), "map_builder_service node stopped");

    return 0;
}
