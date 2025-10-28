#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class MapBuilderTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        rclcpp::init(0, nullptr);
    }

    void TearDown() override
    {
        rclcpp::shutdown();
    }
};

TEST_F(MapBuilderTest, BasicPointCloudConversion)
{
    // Create a simple point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::PointXYZ point;
    point.x = 1.0;
    point.y = 2.0;
    point.z = 3.0;
    cloud->points.push_back(point);
    
    cloud->width = 1;
    cloud->height = 1;
    cloud->is_dense = true;

    // Convert to ROS message
    sensor_msgs::msg::PointCloud2 ros_cloud;
    pcl::toROSMsg(*cloud, ros_cloud);

    // Convert back to PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr converted_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(ros_cloud, *converted_cloud);

    // Verify conversion
    ASSERT_EQ(converted_cloud->points.size(), 1);
    EXPECT_FLOAT_EQ(converted_cloud->points[0].x, 1.0);
    EXPECT_FLOAT_EQ(converted_cloud->points[0].y, 2.0);
    EXPECT_FLOAT_EQ(converted_cloud->points[0].z, 3.0);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}