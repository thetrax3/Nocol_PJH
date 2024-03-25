#include <pcl/io/pcd_io.h>

class PCDViewer
{
public:
    void CreateViewer(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string &viewerName, bool create_interactor = true);
    void CreateViewer_RGB(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const std::string &viewerName, bool create_interactor = true);
    void PlayViewer(int viewerIdx = 0);
    void PlayViewer(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void PlayViewer(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud, const std::string &str);

private:
    void keyboardEventOccurredWrapper(const pcl::visualization::KeyboardEvent &event, void *viewer_void);
    void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event);

private:
    std::vector<pcl::visualization::PCLVisualizer::Ptr> viewers_;
    int current_viewer_index_ = 0;
};