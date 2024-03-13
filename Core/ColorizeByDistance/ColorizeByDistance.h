#include <pcl/io/pcd_io.h>

namespace ColorizeByDistance
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorizePointCloudByDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
}