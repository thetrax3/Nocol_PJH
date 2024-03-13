#include <pcl/filters/voxel_grid.h>

#include "VoxelFilter.h"

void VoxelFilter::PointCloudWithVoxelFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(2.5,2.5,2.5); // Adjust leaf size as needed
    //voxel_filter.setLeafSize(VoxelFilter::leafSize, leafSize, leafSize); // Adjust leaf size as needed
    voxel_filter.filter(*cloud);
}