#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main() {
    pcl::PointCloud<pcl::PointXYZRGB> cloud;

    pcl::PointXYZRGB point;
    point.x = 1.0;
    point.y = 1.0;
    point.z = 1.0;

    point.r = 255;
    point.g = 255;
    point.b = 1.0;


    cloud.push_back(point);

    std::string path = "/home/ngoclong/ros2_ws/src/point_cloud_process/point_clouds/circular_cloud.pcd";
    pcl::io::savePCDFileASCII(path, cloud);


    return 0;
}