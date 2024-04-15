#include <chrono>
#include <memory>
#include <string>
#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/voxel_grid.h"


class KittiVoxelNode: public rclcpp::Node {
public:
    KittiVoxelNode(): Node("kitti_voxel") {
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("voxel_cloud", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                                "/kitti/point_cloud", 10, std::bind(&KittiVoxelNode::timer_callback, this, std::placeholders::_1));
    
        RCLCPP_INFO(this->get_logger(), "Kitti voxel node has been started!");
    }

private:
    void timer_callback(const sensor_msgs::msg::PointCloud2::SharedPtr input_cloud) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*input_cloud, *pcl_cloud);

        // Voxel filter
        pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_cloud (new pcl::PointCloud<pcl::PointXYZ>) ;
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(pcl_cloud);
        voxel_filter.setLeafSize(0.1, 0.1, 0.1);
        voxel_filter.filter(*voxel_cloud);
        
        // Convert cloud to ros2 messages
        sensor_msgs::msg::PointCloud2 voxel_cloud_ros2;
        pcl::toROSMsg(*voxel_cloud, voxel_cloud_ros2);
        voxel_cloud_ros2.header = input_cloud->header;

        std::cout << "PointCloud size before voxelization: " << pcl_cloud->size() << std::endl;
        std::cout << "PointCloud size after voxelization: " << voxel_cloud->size() << std::endl;

        publisher_->publish(voxel_cloud_ros2);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    size_t count_ = 0;
    
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KittiVoxelNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}