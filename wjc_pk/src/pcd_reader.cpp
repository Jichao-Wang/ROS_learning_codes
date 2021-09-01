#include <iostream>
#include <math.h>
#include <time.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
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


int main(int argc, char **argv){
    clock_t startTime,endTime; startTime = clock();

    pcl::PointCloud<pcl::PointXYZI>::Ptr  cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr  distance_fileter_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  visualize_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    
    if (pcl::io::loadPCDFile<pcl::PointXYZI>("crc_short.pcd", *cloud) == -1){
        PCL_ERROR ("can't read file crc_short.pcd \n");
        return (-1);
    }

    std::cout << "loaded " << cloud->width * cloud->height << "data points from the pcd file." << std::endl;

    for(size_t i = 0; i < cloud->size(); ++i){
        float distance = Euclidean_distance(cloud->points[i]);
        
        if(distance <= 4.0 
        && abs(cloud->points[i].z)<0.5 
        && cloud->points[i].intensity>140.0 
        && 150.0>=cloud->points[i].intensity){
            distance_fileter_cloud->push_back(cloud->points[i]);
            /*std::cout << " " << cloud->points[i].x
              << " " << cloud->points[i].y
              << " " << cloud->points[i].z
              << " " << cloud->points[i].intensity
              << " " << distance << std::endl;*/
        }
        /*if(5.0 > cloud->points[i].intensity || cloud->points[i].intensity > 2.0){
            distance_fileter_cloud->push_back(cloud->points[i]);
        }*/
    }

    endTime = clock();
    std::cout << "Totle Time : " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << std::endl;
    std::cout << "loaded " << cloud->width * cloud->height << "data points from the pcd file." << std::endl;
    std::cout << "loaded " << distance_fileter_cloud->width * distance_fileter_cloud->height << "data points from distance_fileter_cloud." << std::endl;
    
    /*3D circle extraction*/
    std::vector<int> inliersCircle3D;
    pcl::SampleConsensusModelCircle3D<pcl::PointXYZI>::Ptr model_circle3D(new pcl::SampleConsensusModelCircle3D<pcl::PointXYZI>(distance_fileter_cloud));
    model_circle3D->setRadiusLimits(0.065,0.07);
    pcl::RandomSampleConsensus<pcl::PointXYZI> ransac(model_circle3D);
    //ransac.setDistanceThreshold(.01);
    ransac.setDistanceThreshold(.05);
    ransac.computeModel();
    ransac.getInliers(inliersCircle3D);
    Eigen::VectorXf modelParas;
    ransac.getModelCoefficients(modelParas);
    std::cout <<"center x,y,z; R; vector"<<modelParas<< "\n\n";
    //std::cout << modelParas[0]<< "\n\n";
    
    /*label points in the circle*/
    pcl::PointXYZI circle_center;
    circle_center.x=modelParas[0];
    circle_center.y=modelParas[1];
    circle_center.z=modelParas[2];
    for(size_t i = 0; i < distance_fileter_cloud->size(); ++i){
        pcl::PointXYZI a;
        a.x=distance_fileter_cloud->points[i].x;
        a.y=distance_fileter_cloud->points[i].y;
        a.z=distance_fileter_cloud->points[i].z;
        if(Euclidean_distance(a,circle_center)<modelParas[3]*1.0){
            distance_fileter_cloud->points[i].intensity=255;
        }
    }

    /*visulization*/
    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> point_cloud_color_handler(distance_fileter_cloud, "intensity");
    viewer.showCloud (distance_fileter_cloud); //visualize the point clouds after filtered
    while (!viewer.wasStopped()){
    }
    return 0;
}
