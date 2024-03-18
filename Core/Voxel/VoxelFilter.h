#include <pcl/io/pcd_io.h>

namespace VoxelFilter
{
    void PointCloudWithVoxelFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, float leafSize = 2.5f);
}