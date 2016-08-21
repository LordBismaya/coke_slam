/**
* This file is part of COKE_SLAM.
*
* Copyright (C) 2016 Bismaya Sahoo <bsahoo at uwaterloo dot ca> (University of Waterloo)
* 
* COKE_SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* COKE_SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with COKE_SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef __COKETRACKING_H
#define __COKETRACKING_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>
#include <stdio.h>
#include <stack>
#include <ctime>

//chadir
#include <unistd.h>
// reading a text file
#include <iostream>
#include <fstream>
#include <string>
//directorio
#include <dirent.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>


//#include <dpptam/SemiDenseMapping.h>
using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"

//#include <tf/transform_broadcaster.h>

class CokeTracking {
  public:
    CokeTracking();

    cv::Mat image_rgb,image_gray,image_crctd,image_grad;
    cv::Mat distCoeffs,cameraMatrix;

    
    
    int frame_count=0;
    
    double fx,fy,cx,cy;
 
//    tf::TransformBroadcaster mTfBr;

	int pyramid_levels;

    vector<cv::Mat> image_pyramid;

    void init_image_pyramid(int);

    cv::Mat *image_frame;
    ros::Time *stamps_ros;

    float depth;
    int reduction;

private:

};

///Semidense tracker thread
void ThreadCokeTracker(CokeTracking *coke_tracker, ros::Publisher *odom_pub,ros::Publisher *pub_poses,ros::Publisher *vis_pub,image_transport::Publisher *pub_image);

///semidense_tracking function
void coke_tracking(CokeTracking *coke_tracker, ros::Publisher *odom_pub,ros::Publisher *pub_poses, ros::Publisher *vis_pub,image_transport::Publisher *pub_image);

///prepare the image for tracking
void prepare_image(cv::Mat &image_frame, cv::Mat &image_rgb,int &frame_count,cv::Mat &image_gray,cv::Mat cameraMatrix,cv::Mat distCoeffs,double &fx,double &fy, double &cx,double &cy, int reduction);

#endif
