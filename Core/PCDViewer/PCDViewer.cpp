#include <pcl/visualization/pcl_visualizer.h>
#include <chrono>
#include <pcl/common/transforms.h>
#include <Windows.h>

#include "PCDViewer.h"

bool isKeyPressed(int keyCode)
{
    // 키의 가장 상위 비트를 사용하여 키의 상태 확인
    return GetAsyncKeyState(keyCode) & 0x8000;
}

bool keyboardFunctionInViewer(std::chrono::steady_clock::time_point &start, std::chrono::steady_clock::time_point &end, Eigen::AngleAxisf &rotateAngle,
                              long long &elapsed, double fps, double &rotateSpeed, int &frames)
{
    bool keyPressed = false;

    frames++;
    end = std::chrono::steady_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    if (elapsed >= 1000)
    {
        fps = frames * 1000.0 / elapsed;
        frames = 0;
        start = end;
    }
    if (isKeyPressed(VK_NUMPAD1))
    {
        rotateSpeed = 10;
    }
    if (isKeyPressed(VK_NUMPAD2))
    {
        rotateSpeed = 50;
    }
    if (isKeyPressed(VK_NUMPAD3))
    {
        rotateSpeed = 100;
    }
    if (isKeyPressed(VK_NUMPAD4))
    {
        rotateSpeed = 1000;
    }
    if (isKeyPressed(VK_UP))
    {
        rotateAngle = Eigen::AngleAxisf(rotateSpeed * fps, Eigen::Vector3f::UnitX());
        keyPressed = true;
    }
    if (isKeyPressed(VK_DOWN))
    {
        rotateAngle = Eigen::AngleAxisf(-rotateSpeed * fps, Eigen::Vector3f::UnitX());
        keyPressed = true;
    }
    if (isKeyPressed(VK_LEFT))
    {
        rotateAngle = Eigen::AngleAxisf(rotateSpeed * fps, Eigen::Vector3f::UnitY());
        keyPressed = true;
    }
    if (isKeyPressed(VK_RIGHT))
    {
        rotateAngle = Eigen::AngleAxisf(-rotateSpeed * fps, Eigen::Vector3f::UnitY());
        keyPressed = true;
    }
    return keyPressed;
}

void PCDViewer::PlayViewer(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::visualization::PCLVisualizer viewer("PCD Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud");
    viewer.addSphere(pcl::PointXYZ{0.0, 0.0, 0.0}, 1.0, 255.0, 0.0, 0.0, "center");

    if (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
}

void PCDViewer::PlayViewer(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud, const std::string &viewerName)
{

    double rotateSpeed = 10;
    double fps = 0;
    auto start = std::chrono::steady_clock::now();
    auto end = std::chrono::steady_clock::now();
    long long elapsed = 0;
    int frames = 0;
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    Eigen::AngleAxisf rotateAngle;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(viewerName));
    viewer->setBackgroundColor(0.0, 0.0, 0.0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(colored_cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(colored_cloud, rgb, viewerName);
    viewer->addSphere(pcl::PointXYZ{0.0, 0.0, 0.0}, 1.0, 255.0, 0.0, 0.0, "center");

    while (!viewer->wasStopped())
    {
        /**
         * TODO:
         * 키보드 입력으로 pcd 회전 기능
         * 이후 왜곡률 보정 기능으로 수정할 것
         */
        if (keyboardFunctionInViewer(start, end, rotateAngle, elapsed, fps, rotateSpeed, frames))
        {
            transform.rotate(rotateAngle);
            pcl::transformPointCloud(*colored_cloud, *colored_cloud, transform);
            viewer->updatePointCloud(colored_cloud);
            std::cout << "rotation : " << transform.rotation() << std::endl;
            std::cout << "rotateSpeed : " << rotateSpeed << ", fps : " << fps << std::endl;
        }

        viewer->spinOnce();
    }
}

void PCDViewer::CreateViewer(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string &viewerName, bool create_interactor)
{
    auto viewer = new pcl::visualization::PCLVisualizer(viewerName, create_interactor);
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud(cloud, viewerName);
    viewer->registerKeyboardCallback(&PCDViewer::keyboardEventOccurredWrapper, *this);

    viewers_.emplace_back(viewer);
    assert(viewers_.size() != 0);
}

void PCDViewer::CreateViewer_RGB(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const std::string &viewerName, bool create_interactor)
{
    auto viewer = new pcl::visualization::PCLVisualizer(viewerName, create_interactor);
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud(cloud, viewerName);
    viewer->registerKeyboardCallback(&PCDViewer::keyboardEventOccurredWrapper, *this);

    viewers_.emplace_back(viewer);
    assert(viewers_.size() != 0);
}

void PCDViewer::PlayViewer(int viewerIdx)
{
    current_viewer_index_ = viewerIdx;
    static int last_Index = -1;

    while (!viewers_.empty())
    {
        if (last_Index != current_viewer_index_)
        {
            if (last_Index == -1)
            {
                last_Index = current_viewer_index_;
            }
            else if (!viewers_[last_Index]->wasStopped())
            {
                viewers_[last_Index]->close();
                last_Index = current_viewer_index_;
            }

            if (current_viewer_index_ > viewers_.size())
            {
                current_viewer_index_ = viewers_.size() - 1;
            }

            assert(last_Index >= 0 && last_Index < viewers_.size());
            viewers_[last_Index]->createInteractor();
            viewers_[last_Index]->setSize(800, 600);
        }

        if (viewers_[last_Index]->wasStopped())
        {
            for (auto viewer : viewers_)
            {
                viewer->close();
            }
            break;
        }

        viewers_[last_Index]->spinOnce(100);
    }
}

void PCDViewer::keyboardEventOccurredWrapper(const pcl::visualization::KeyboardEvent &event, void *viewer_void)
{
    this->keyboardEventOccurred(event);
}

void PCDViewer::keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event)
{
    if (event.getKeyCode() >= '1' && event.getKeyCode() <= '9')
    {
        int viewer_index = event.getKeyCode() - '1';

        if (viewer_index >= 0 && viewer_index < viewers_.size())
        {
            current_viewer_index_ = viewer_index;
            std::cout << "Switched to Viewer" << viewer_index + 1 << std::endl;
        }
    }
    // else{
    //     switch(event.getKeyCode()){
    //         case 'w':
    //             break;
    //     }
    // }
}