#include <iostream>
#include <math.h>
#include <time.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/sample_consensus/sac_model_circle3d.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/segmentation/progressive_morphological_filter.h>


float Euclidean_distance(float x, float y, float z){
    return sqrt(pow(x,2)+pow(y,2)+pow(z,2));
}

float Euclidean_distance(pcl::PointXYZI p){
    return sqrt(pow(p.x,2)+pow(p.y,2)+pow(p.z,2));
}

float Euclidean_distance(pcl::PointXYZI a, pcl::PointXYZI b){
    return sqrt(pow(a.x-b.x,2)+pow(a.y-b.y,2)+pow(a.z-b.z,2));
}





typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

boost::mutex cloud_mutex;//进程锁

// 用于传给回调函数的结构体定义
struct callback_args{
    PointCloudT::Ptr clicked_points_3d;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewerPtr;
};

// 回调函数
void pp_callback(const pcl::visualization::PointPickingEvent& event, void* args){
    struct callback_args* data = (struct callback_args*)args;
    if (event.getPointIndex() == -1)
        return;
    PointT current_point;
    event.getPoint(current_point.x, current_point.y, current_point.z);
    data->clicked_points_3d->clear();//将上次选的点清空
    data->clicked_points_3d->points.push_back(current_point);//添加新选择的点
    // 设置屏幕渲染属性，红色显示选择的点
    pcl::visualization::PointCloudColorHandlerCustom<PointT> red(data->clicked_points_3d, 255, 0, 0);
    data->viewerPtr->removePointCloud("clicked_points");
    data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
    data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
    std::cout <<"点的坐标为:"<< "x="<<current_point.x << "y= " << current_point.y << "z= " << current_point.z << std::endl;
}

// 屏幕上拾取点
void pointpicking(pcl::PointCloud< PointT>::Ptr cloud){
    cloud_mutex.lock();   // 获得互斥体，期间不能修改点云
    // 可视化点云
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));
    //viewer->setBackgroundColor(255,255,255);
    viewer->addPointCloud<PointT>(cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud"); // 设置点云大小
    //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "sample cloud");
    viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
    viewer->setWindowName("查看坐标");

    // 给可视化窗口添加选择点的回调函数(Add point picking callback to viewer):
    struct callback_args cb_args;
    PointCloudT::Ptr clicked_points_3d(new PointCloudT);
    cb_args.clicked_points_3d = clicked_points_3d;
    cb_args.viewerPtr = viewer;
    //注册屏幕选择事件
    viewer->registerPointPickingCallback(pp_callback, (void*)&cb_args);
    //Shift+鼠标左键选择点，按Q结束
    std::cout << "Shift+click on three floor points, then press 'Q'..." << std::endl;

    viewer->spin();
    std::cout << "done." << std::endl;

    cloud_mutex.unlock();//释放互斥体

    while (!viewer->wasStopped()){
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}

int main(){
    std::string filename("crc_short.pcd");
    pcl::PointCloud< PointT>::Ptr cloud(new pcl::PointCloud< PointT>());
    if (pcl::io::loadPCDFile(filename, *cloud)){
        std::cerr << "ERROR: Cannot open file " << filename << "! Aborting..." << std::endl;
        return -1;
    }
    std::cout << "读取点云的个数为" << cloud->points.size() << std::endl;
    pointpicking(cloud);
    return 0;
}
