#include <pcl/point_types.h>
#include "ColorizeByDistance.h"

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColorizeByDistance::colorizePointCloudByDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    double max_distance = 0.0;
    double *distanceArr = new double[cloud->points.size()];

    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        double distance = sqrt(cloud->points[i].x * cloud->points[i].x +
                               cloud->points[i].y * cloud->points[i].y +
                               cloud->points[i].z * cloud->points[i].z);
        if (distance > max_distance)
            max_distance = distance;
        distanceArr[i] = distance;
    }

    // 포인트 클라우드 시각화에 사용할 컬러값 계산
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    colored_cloud->points.resize(cloud->points.size());

    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        double r = distanceArr[i] / max_distance;         // Red component based on distance
        double g = 0.0;                                   // Green component (for simplicity, set to 0)
        double b = 1.0 - (distanceArr[i] / max_distance); // Blue component based on distance

        uint8_t r_byte = static_cast<uint8_t>(255.0 * r);
        uint8_t g_byte = static_cast<uint8_t>(255.0 * g);

        uint8_t b_byte = static_cast<uint8_t>(255.0 * b);

        colored_cloud->points[i].x = cloud->points[i].x;
        colored_cloud->points[i].y = cloud->points[i].y;
        colored_cloud->points[i].z = cloud->points[i].z;
        colored_cloud->points[i].r = r_byte;
        colored_cloud->points[i].g = g_byte;
        colored_cloud->points[i].b = b_byte;
    }
    delete[] distanceArr;
    // 포인트 클라우드 시각화

    return colored_cloud;
}