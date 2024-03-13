#include <pcl/io/pcd_io.h>

namespace PCDViewer
{
    void PlayViewer(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void PlayViewer(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud);
};