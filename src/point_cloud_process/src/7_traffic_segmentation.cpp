#include <chrono>
#include <memory>
#include <string>
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/voxel_grid.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "visualization_msgs/msg/marker_array.hpp"
#include <pcl/filters/passthrough.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree.h>

#include <pcl/point_cloud.h>
using namespace std::chrono_literals;
typedef pcl::PointXYZ PointT;

class VoxelGrid_filter : public rclcpp::Node
{
  public:
    VoxelGrid_filter()
    : Node("minimal_publisher")
    {
      subscription_ =
      this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/kitti/point_cloud", 10, std::bind(&VoxelGrid_filter::timer_callback, this, std::placeholders::_1));

      publisher_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("voxel_cloud", 10);
    }

  private:
    void timer_callback(const sensor_msgs::msg::PointCloud2::SharedPtr input_cloud)
      {
        pcl::PointCloud<PointT>::Ptr pcl_cloud (new pcl::PointCloud<PointT>) ;
        pcl::PointCloud<PointT>::Ptr cropped_cloud (new pcl::PointCloud<PointT>) ;

        pcl::fromROSMsg(*input_cloud, *pcl_cloud);
  //==================================== Pre Processing Data ====================================
        pcl::PassThrough<PointT> passing_x;
        pcl::PassThrough<PointT> passing_y;
        int radius = 15;
        passing_x.setInputCloud(pcl_cloud);
        passing_x.setFilterFieldName("x");
        passing_x.setFilterLimits(-radius,radius);
        passing_x.filter(*cropped_cloud);

        // Along Y Axis

        passing_y.setInputCloud(cropped_cloud);
        passing_y.setFilterFieldName("y");
        passing_y.setFilterLimits(-radius,radius);
        passing_y.filter(*cropped_cloud);
        // Voxel Filter
        pcl::PointCloud<PointT>::Ptr voxel_cloud (new pcl::PointCloud<PointT>) ;
        pcl::VoxelGrid<PointT> voxel_filter;
        voxel_filter.setInputCloud(cropped_cloud);
        voxel_filter.setLeafSize(0.1 , 0.1, 0.1);
        voxel_filter.filter(*voxel_cloud);
  //==================================== Road Segmentation  ====================================
        pcl::NormalEstimation<PointT, pcl::Normal> normal_extractor;
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
        pcl::PointCloud<pcl::Normal>::Ptr road_normals(new pcl::PointCloud<pcl::Normal>);

        pcl::SACSegmentationFromNormals<PointT, pcl::Normal> road_seg_frm_normals;
        pcl::PointIndices::Ptr road_inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr road_coefficients(new pcl::ModelCoefficients);
        pcl::ExtractIndices<PointT> road_extract_indices;
        pcl::PointCloud<PointT>::Ptr road_cloud(new pcl::PointCloud<PointT>);


        // Normals Extractions
        normal_extractor.setSearchMethod(tree);
        normal_extractor.setInputCloud(voxel_cloud);
        normal_extractor.setKSearch(30);
        normal_extractor.compute(*road_normals);

        // Parameters for Planar Segmentation
        road_seg_frm_normals.setOptimizeCoefficients(true);
        road_seg_frm_normals.setModelType(pcl::SACMODEL_NORMAL_PLANE);
        road_seg_frm_normals.setMethodType(pcl::SAC_RANSAC);
        road_seg_frm_normals.setNormalDistanceWeight(0.5);
        road_seg_frm_normals.setMaxIterations(100);
        road_seg_frm_normals.setDistanceThreshold(0.4);
        road_seg_frm_normals.setInputCloud(voxel_cloud);
        road_seg_frm_normals.setInputNormals(road_normals);
        road_seg_frm_normals.segment(*road_inliers,*road_coefficients);

        //Extracting Cloud based on Inliers indices
        road_extract_indices.setInputCloud(voxel_cloud);
        road_extract_indices.setIndices(road_inliers);
        road_extract_indices.setNegative(true);
        road_extract_indices.filter(*road_cloud);
  //==================================== Traffic Segmentation  ====================================
    pcl::PointCloud<PointT>::Ptr segmented_cluster (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr all_clusters (new pcl::PointCloud<PointT>);
    tree->setInputCloud (road_cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;

    ec.setClusterTolerance (0.25); 
    ec.setMinClusterSize (600);
    ec.setMaxClusterSize (2000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (road_cloud);
    ec.extract (cluster_indices);

    size_t min_reasonable_size = 610;
    size_t max_reasonable_size = 1900;
    for (size_t i = 0; i < cluster_indices.size(); i++)
    {
        if (cluster_indices[i].indices.size() > min_reasonable_size && cluster_indices[i].indices.size() < max_reasonable_size)
        {
            pcl::PointCloud<PointT>::Ptr reasonable_cluster (new pcl::PointCloud<PointT>);
            pcl::ExtractIndices<PointT> extract;
            pcl::IndicesPtr indices(new std::vector<int>(cluster_indices[i].indices.begin(), cluster_indices[i].indices.end()));
            extract.setInputCloud (road_cloud);
            extract.setIndices(indices);
            extract.setNegative (false);
            extract.filter (*reasonable_cluster);
            all_clusters->operator+=(*reasonable_cluster);
        }
    }

  //==================================== Cloud publishing to ROS  ====================================

        // Convert cloud to ros2 message
        sensor_msgs::msg::PointCloud2 traffic_seg_ros2;
        pcl::toROSMsg(*all_clusters, traffic_seg_ros2);
        traffic_seg_ros2.header = input_cloud->header;

        publisher_->publish(traffic_seg_ros2);


  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VoxelGrid_filter>());
  rclcpp::shutdown();
  return 0;
}