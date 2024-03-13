#include <pcl/visualization/pcl_visualizer.h>

#include "PCDViewer.h"

void PCDViewer::PlayViewer(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::visualization::PCLVisualizer viewer("PCD Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud");
    viewer.addSphere(pcl::PointXYZ{0.0, 0.0, 0.0}, 1.0, 255.0, 0.0, 0.0, "center");

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
}

void PCDViewer::PlayViewer(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud)
{
    pcl::visualization::PCLVisualizer viewer("PCD Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(colored_cloud);
    viewer.addPointCloud<pcl::PointXYZRGB>(colored_cloud, rgb, "colored_cloud");
    viewer.addSphere(pcl::PointXYZ{0.0, 0.0, 0.0}, 1.0, 255.0, 0.0, 0.0, "center");

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
}