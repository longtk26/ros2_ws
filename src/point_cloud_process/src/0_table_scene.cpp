#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>


int main() {
    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
    pcl::PCDReader cloud_reader;

    std::string path = "/home/ngoclong/ros2_ws/src/point_cloud_process/point_clouds/table_scene.pcd";
    cloud_reader.read(path, *cloud);

    std::cout << "Loaded "
              << cloud->width * cloud->height
              << " data points from test_pcd.pcd with the following fields: "
              << std::endl;

    return 0;
}