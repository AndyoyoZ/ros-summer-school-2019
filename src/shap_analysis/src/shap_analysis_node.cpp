#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <Eigen/Core>

#include <opencv2/opencv.hpp>

#include <iostream>


int img_w = 720;
double focal = 500;
double z = 1;   

//laserScan to Eigen::Vector3d
void TranScanToPoints3D(const sensor_msgs::LaserScan& scan_in,std::vector<Eigen::Vector3d>& Points)
{
    // http://wiki.ros.org/laser_geometry
    size_t n_pts = scan_in.ranges.size ();
    Eigen::ArrayXXd ranges (n_pts, 2);
    Eigen::ArrayXXd output (n_pts, 2);
    Eigen::ArrayXXd co_sine_map (n_pts, 2);

    for (size_t i = 0; i < n_pts; ++i)
    {
        ranges (i, 0) = (double) scan_in.ranges[i];
        ranges (i, 1) = (double) scan_in.ranges[i];

        co_sine_map (i, 0) = cos (scan_in.angle_min + (double) i * scan_in.angle_increment);
        co_sine_map (i, 1) = sin (scan_in.angle_min + (double) i * scan_in.angle_increment);
    }

    output = ranges * co_sine_map;

    for (size_t i = 0; i < n_pts; ++i) {

        // TODO: range_cutoff threashold
        double range_cutoff = 30.0;
        const float range = scan_in.ranges[i];
        if (range < range_cutoff && range >= scan_in.range_min)
        {
            Points.push_back(Eigen::Vector3d(output (i, 0), output (i, 1), 0) );
        } else
        {
            Points.push_back(Eigen::Vector3d(1000.0,1000.0, 0) );
        }

    }

}


//laserScan to cv::Point
void TranScanToPoints2D(const sensor_msgs::LaserScan& scan_in,std::vector<cv::Point>& Points)
{
    // http://wiki.ros.org/laser_geometry
    size_t n_pts = scan_in.ranges.size ();
    Eigen::ArrayXXd ranges (n_pts, 2);
    Eigen::ArrayXXd output (n_pts, 2);
    Eigen::ArrayXXd co_sine_map (n_pts, 2);

    for (size_t i = 0; i < n_pts; ++i)
    {
        ranges (i, 0) = (double) scan_in.ranges[i];
        ranges (i, 1) = (double) scan_in.ranges[i];

        co_sine_map (i, 0) = cos (scan_in.angle_min + (double) i * scan_in.angle_increment);
        co_sine_map (i, 1) = sin (scan_in.angle_min + (double) i * scan_in.angle_increment);
    }

    output = ranges * co_sine_map;

    for (size_t i = 0; i < n_pts; ++i) {

        // TODO: range_cutoff threashold
        double range_cutoff = 30.0;
        const float range = scan_in.ranges[i];
        if (range < range_cutoff && range >= scan_in.range_min)
        {
            Points.push_back(cv::Point( (int)(output (i, 0) / z * focal + img_w/2)  , (int)(output (i, 1) / z * focal + img_w/2)) );
        } 

    }

}


//三维点云转image
//可以将三维点直接投影为二维点，直接进行多边形逼近，得到物体形状

void PointsToImg(const std::vector<Eigen::Vector3d> points, cv:: Mat &dst)
{

    
    cv:: Mat  img(img_w, img_w, CV_8UC1, cv::Scalar::all(0));

    for (auto pt: points) {
        int col = (int)(pt.x() / z * focal + img_w/2);
        int row = (int)(- pt.y() / z * focal + img_w/2);  // -Y/Z 加了一个负号, 是为了抵消针孔投影时的倒影效果

        if(col > img_w-1 || col< 0 || row > img_w-1 || row < 0)
            continue;

        img.at<uchar>(row, col) = 255;
    }
    img.copyTo(dst);
}





void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    std::vector<cv::Point> points2d;
    std::vector<Eigen::Vector3d> points3d;
    std::vector<cv::Point> approxCurve;

    // test
    TranScanToPoints3D(*msg,points3d);
    cv:: Mat  img(img_w, img_w, CV_8UC1, cv::Scalar::all(0));
    PointsToImg(points3d,img);
    cv::imshow("img",img);
    cv::waitKey(10);


    TranScanToPoints2D(*msg,points2d);
    int epsilon=0.0115*cv::arcLength(points2d,true);
    std::cout<<"epsilon = "<<epsilon<<std::endl;
    cv::approxPolyDP(points2d,approxCurve,epsilon,true);
    std::cout<<"shap is ";
    int corners = approxCurve.size();
    if(3==corners)
    {
        std::cout<<"triangle"<<std::endl;
    }
    else if(4==corners)
    {
        std::cout<<"rectangle"<<std::endl;
    }
    else if(9<corners)
    {
        std::cout<<"circle"<<std::endl;
    }
    else
    {
        std::cout<<corners<<"-polygon"<<std::endl;
    }

}



int main(int argc, char **argv)
{
    std::string scan_topic_name = "/scan";

    ros::init(argc, argv, "LaserCamCalibra");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe(scan_topic_name, 1, laserCallback);
    ros::spin();

    ros::shutdown();
    return 0;
    
}