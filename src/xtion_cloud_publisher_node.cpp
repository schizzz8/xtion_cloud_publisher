#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Core>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

using namespace std;

float depth_scale = 0.001;
float max_distance = 5.0;
float min_distance = 0.4;
bool gotInfo = false;

ros::Subscriber info_sub;
ros::Subscriber frame_sub;
ros::Publisher cloud_pub;

sensor_msgs::CameraInfo camerainfo;

Eigen::Matrix3f K;
Eigen::Matrix3f invK;

void infoCallback(const sensor_msgs::CameraInfo::ConstPtr& info){
    //K matrix
    //================================================================================
    camerainfo.K = info->K;
    ROS_INFO("Got camera info!");
    K(0,0) = camerainfo.K.c_array()[0];
    K(0,1) = camerainfo.K.c_array()[1];
    K(0,2) = camerainfo.K.c_array()[2];
    K(1,0) = camerainfo.K.c_array()[3];
    K(1,1) = camerainfo.K.c_array()[4];
    K(1,2) = camerainfo.K.c_array()[5];
    K(2,0) = camerainfo.K.c_array()[6];
    K(2,1) = camerainfo.K.c_array()[7];
    K(2,2) = camerainfo.K.c_array()[8];

    invK=K.inverse();
    cerr << "got camera matrix" << endl;
    cerr << K << endl;

    //Don't want to receive any camera info stuff, basically this callback is used a init function
    info_sub.shutdown();
    //set flag. Bye.
    gotInfo=true;
}


void frameCallback(const sensor_msgs::ImageConstPtr& depth_msg) {

    if (!gotInfo) return;

    // Create a PointCloud2
    sensor_msgs::PointCloud2 cloud_msg;
    cloud_msg.header = depth_msg->header;
    cloud_msg.height = depth_msg->height;
    cloud_msg.width  = depth_msg->width;
    cloud_msg.is_dense = false;
    sensor_msgs::PointCloud2Modifier pcd_modifier(cloud_msg);
    pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

    float bad_point = std::numeric_limits<float>::quiet_NaN();
    int count = 0, good = 0, bad = 0;

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

    cv_bridge::CvImageConstPtr image =  cv_bridge::toCvShare(depth_msg);
    for(int i =0;i<image->image.rows;i++){
        const ushort* row_ptr = image->image.ptr<ushort>(i);
        for(int j=0;j<image->image.cols;j++,++iter_x,++iter_y,++iter_z){
            ushort id=row_ptr[j];
            if(id!=0){
                good++;
                float d=depth_scale*id;
                Eigen::Vector3f image_point(j*d,i*d,d);
                Eigen::Vector3f camera_point=invK*image_point;
                *iter_x = camera_point.x();
                *iter_y = camera_point.y();
                *iter_z = camera_point.z();
                //std::cerr << camera_point.transpose() << "\t";
            }else{
                bad++;
                *iter_x = *iter_y = *iter_z = bad_point;
                continue;
            }
            count++;
        }
    }
    cloud_pub.publish(cloud_msg);
    std::cerr << ".";
    //std::cerr << count << "=" << good << "+" << bad << "\t";
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    //Subscribers
    //================================================================================
    frame_sub = nh.subscribe("/xtion/depth/image_raw", 1, frameCallback);
    info_sub = nh.subscribe("/xtion/depth/camera_info", 1, infoCallback);
    //Publishers
    //================================================================================
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud", 1);


    ros::spin();
    cv::destroyWindow("view");
}
