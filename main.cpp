#include <iostream>
#include <chrono>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>

#include "Core/FileDialog/FileDialog.h"
#include "Core/Voxel/VoxelFilter.h"
#include "Core/PCDViewer/PCDViewer.h"

void FileOpen(std::string &targetString)
{
    // 1. 파일 탐색기로 경로 획득
    targetString = FileDialog::openFileDialog();

    auto start_chrono = std::chrono::high_resolution_clock::now();

    if (!targetString.empty())
    {
        std::cout << "Selected file path: " << targetString << std::endl;
    }
    else
    {
        std::cerr << "No file selected or operation canceled." << std::endl;
        return;
    }
    auto end_chrono = std::chrono::high_resolution_clock::now();
    std::cout << "File Exitst!\nElapse Time_FileExistCheck : " << std::chrono::duration_cast<std::chrono::milliseconds>(end_chrono - start_chrono).count() << std::endl;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreatePCD(std::string &path)
{
    // 2. 포인트 클라우드 객체 생성
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // 3. PCD 파일에서 데이터 읽기
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file %s\n", path.c_str());
        return nullptr;
    }

    return cloud;
}

int main()
{
    auto start_chrono = std::chrono::high_resolution_clock::now();
    std::string pcdFilePath;
    FileOpen(pcdFilePath);
    auto end_chrono = std::chrono::high_resolution_clock::now();
    std::cout << "Elapse File_Open : " << std::chrono::duration_cast<std::chrono::milliseconds>(end_chrono - start_chrono).count() << std::endl;

    start_chrono = std::chrono::high_resolution_clock::now();
    auto cloud = CreatePCD(pcdFilePath);
    end_chrono = std::chrono::high_resolution_clock::now();
    std::cout << "Elapse CreatePCD : " << std::chrono::duration_cast<std::chrono::milliseconds>(end_chrono - start_chrono).count() << std::endl;

    std::cout << "Before Cloud count : " << cloud->points.size() << std::endl;
    start_chrono = std::chrono::high_resolution_clock::now();
    VoxelFilter::PointCloudWithVoxelFilter(cloud);
    end_chrono = std::chrono::high_resolution_clock::now();
    std::cout << "Elapse Voxel : " << std::chrono::duration_cast<std::chrono::milliseconds>(end_chrono - start_chrono).count() << std::endl;
    std::cout << "After Cloud count : " << cloud->points.size() << std::endl;

    start_chrono = std::chrono::high_resolution_clock::now();
    auto colorized_cloud = ColorizeByDistance::colorizePointCloudByDistance(cloud);
    end_chrono = std::chrono::high_resolution_clock::now();
    std::cout << "Elapse Colorize : " << std::chrono::duration_cast<std::chrono::milliseconds>(end_chrono - start_chrono).count() << std::endl;

    PCDViewer::PlayViewer(colorized_cloud);

    return 0;
}