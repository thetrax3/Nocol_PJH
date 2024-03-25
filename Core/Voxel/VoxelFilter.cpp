#include <pcl/filters/voxel_grid.h>

#include "VoxelFilter.h"

void VoxelFilter::PointCloudWithVoxelFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, float leafSize)
{
    if (cloud == nullptr)
    {
        std::cerr << "Cloud Data Should't nullptr" << std::endl;
    }
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(leafSize, leafSize, leafSize);
    voxel_filter.filter(*cloud);
}