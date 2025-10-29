#include "map_builder/enhanced_visual_odometry.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    try
    {
        auto node = std::make_shared<map_builder::EnhancedVisualOdometry>();
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("enhanced_visual_odometry_node"), 
                     "Exception in enhanced visual odometry node: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}