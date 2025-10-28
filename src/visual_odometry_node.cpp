#include "map_builder/visual_odometry.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<map_builder::VisualOdometry>();
    
    RCLCPP_INFO(node->get_logger(), "Starting Visual Odometry Node");
    
    try
    {
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(node->get_logger(), "Exception in visual odometry node: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}