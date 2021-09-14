#include <iostream>
#include <math.h>
#include <bits/stdc++.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <opencv2/opencv.hpp>  

using namespace std;
using namespace cv;

float h_res = 0.3; /*horizontal beam resolution 0.03*/
float v_res = 0.3; /*vertical beam resolution 0.28*/
float Pi = acos(-1);

class my_point{
    public:
        double x_3d,y_3d,z_3d;
        int x_2d,y_2d;
        double intensity;

        my_point(); /*default initialize all params as 0.*/
        my_point(double a, double b, double c); /*3 params is used to initialize 3D coordinate.*/
        my_point(double a, double b, double c, double d);/*4 params is used to initialize 3D coordinate and intensity.*/
        my_point(pcl::PointXYZ p);
        my_point(pcl::PointXYZI p);
        void init_by_PointXYZI(pcl::PointXYZI p);

        void print_3d_coordinate();
        void print_2d_coordinate();
        void print_intensity();

        void reflect_3d_to_2d();
};

void save_2d_grayscale(vector<my_point> &my_points, int frames){
    /*initialize a 2D matrix*/
    int a;a=360/h_res + 1;int b=180/v_res + 1;
    //cout<<"image size:"<<a<<" "<<b<<" "<<my_points.size()<<endl;
	Mat dstImage = Mat::ones(a,b, CV_8UC1);

    /*draw points*/
    std::cout<<"debug 4"<<std::endl;
    for(int i=0;i<my_points.size();i++){
        //std::cout<<my_points[i].x_2d<<" "<<my_points[i].y_2d<<std::endl;
        dstImage.ptr<Vec3b>(my_points[i].x_2d)[my_points[i].y_2d]=my_points[i].intensity*10;
    }

    /*save the matrix as an image*/
    string s1="./gray_images/gray_image",s2=to_string(frames),s3=".jpg";
    imwrite(s1+s2+s3, dstImage);
}

void my_point::reflect_3d_to_2d(){
    float gamma = atan2(this->y_3d, this->x_3d)/Pi*180;
    float fie = asin(this->z_3d/sqrt(pow(this->x_3d,2)+pow(this->y_3d,2)+pow(this->z_3d,2)))/Pi*180;
    //cout<<"gamma="<<gamma<<" "<<this->x_3d<<" "<<this->y_3d<<" "<<this->z_3d<<", fie="<<fie<<" "<<(this->z_3d/sqrt(pow(this->x_3d,2)+pow(this->y_3d,2)+pow(this->z_3d,2)))<<" "<<sqrt(pow(this->x_3d,2)+pow(this->y_3d,2)+pow(this->z_3d,2))<<endl;
    float mid_x_2d = gamma / h_res;
    float mid_y_2d = fie / v_res;
    char buf[14];
    sprintf(buf,"%.0f",mid_x_2d+180/h_res);
	this->x_2d=atoi(buf);
	sprintf(buf,"%.0f",mid_y_2d+90/v_res);
	this->y_2d=atoi(buf);
}

my_point::my_point(){
    this->x_3d=0;
    this->y_3d=0;
    this->z_3d=0;
    this->x_2d=0;
    this->y_2d=0;
    this->intensity=0;
}

my_point::my_point(double a, double b, double c){
    this->x_3d=a;
    this->y_3d=b;
    this->z_3d=c;
    this->x_2d=0;
    this->y_2d=0;
    this->intensity=0;
    this->reflect_3d_to_2d();
}

my_point::my_point(double a, double b, double c, double d){
    this->x_3d=a;
    this->y_3d=b;
    this->z_3d=c;
    this->x_2d=0;
    this->y_2d=0;
    this->intensity=d;
    this->reflect_3d_to_2d();
}

my_point::my_point(pcl::PointXYZ p){
    this->x_3d=p.x;
    this->y_3d=p.y;
    this->z_3d=p.z;
    this->x_2d=0;
    this->y_2d=0;
    this->intensity=0;
    this->reflect_3d_to_2d();
}

my_point::my_point(pcl::PointXYZI p){
    this->x_3d=p.x;
    this->y_3d=p.y;
    this->z_3d=p.z;
    this->x_2d=0;
    this->y_2d=0;
    this->intensity=p.intensity;
    this->reflect_3d_to_2d();
}

void my_point::init_by_PointXYZI(pcl::PointXYZI p){
    this->x_3d=p.x;
    this->y_3d=p.y;
    this->z_3d=p.z;
    this->x_2d=0;
    this->y_2d=0;
    this->intensity=p.intensity;
    this->reflect_3d_to_2d();
}

void my_point::print_3d_coordinate(){
    cout<<x_3d<<" "<<y_3d<<" "<<z_3d<<std::endl;
}

void my_point::print_2d_coordinate(){
    cout<<x_2d<<" "<<y_2d<<std::endl;
}

void my_point::print_intensity(){
    cout<<intensity<<std::endl;
}