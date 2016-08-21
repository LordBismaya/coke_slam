#ifndef __VO_SYSTEM_H
#define __VO_SYSTEM_H


#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <cstdio>


using namespace std;

// TIC - TOC
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include <math.h>


#include <boost/thread/thread.hpp>
#include <iostream>
#include <stdio.h>
#include <boost/filesystem.hpp>


#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <visualization_msgs/Marker.h>

/////ROS IMAGE SUBSCRIBER
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>

//#include <coke_slam/superpixel.h>
//#include <dpptam/DenseMapping.h>
#include <coke_slam/CokeTracking.h>
//#include <dpptam/SemiDenseMapping.h>

class cs_system
{
public:
    cs_system();


    void imgcb(const sensor_msgs::Image::ConstPtr& msg);


//    DenseMapping dense_mapper;
    CokeTracking coke_tracker;
//    SemiDenseMapping semidense_mapper;
//    MapShared Map;

    int cont_frames;
    double stamps;

    cv::Mat image_frame,image_frame_aux;

    double depth_stamps;

    ros::Time current_time,stamps_ros;

    ros::NodeHandle nh;

    image_transport::Subscriber sub1;

    image_transport::Publisher pub_image;

    ros::Publisher odom_pub;
    ros::Publisher pub_cloud;
    ros::Publisher pub_poses;
    ros::Publisher vis_pub;

};
#endif



