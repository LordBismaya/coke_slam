/**
* This file is part of DPPTAM.
*
* Copyright (C) 2015 Alejo Concha Belenguer <alejocb at unizar dot es> (University of Zaragoza)
* and Javier Civera Sancho   <jcivera at unizar dot es> (University of Zaragoza)
*
* DPPTAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DPPTAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DPPTAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include <coke_slam/CokeTracking.h>
#include <coke_slam/cs_system.h>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <ros/ros.h>
#include <ros/package.h>

#include <image_transport/image_transport.h>


CokeTracking::CokeTracking()
{

   cv::FileStorage  fs2( (ros::package::getPath("coke_slam")+"/src/data.yml").c_str(), cv::FileStorage::READ);
   fs2["cameraMatrix"] >> cameraMatrix;
   fs2["distCoeffs"] >> distCoeffs;
   fs2["pyramids"] >> pyramid_levels;

   frame_count=0;

   reduction = 2;

   fs2.release();
}

void ThreadCokeTracker(CokeTracking *coke_tracker,ros::Publisher *odom_pub,ros::Publisher *pub_poses,ros::Publisher *vis_pub,image_transport::Publisher *pub_image)
{
    while (ros::ok())
    {
           coke_tracking(coke_tracker,odom_pub,pub_poses,vis_pub,pub_image);
           boost::this_thread::sleep(boost::posix_time::milliseconds(1));
    }
}



void coke_tracking(CokeTracking *coke_tracker,ros::Publisher *odom_pub,ros::Publisher *pub_poses,ros::Publisher *vis_pub,image_transport::Publisher *pub_image)
{
    cv::Mat image_frame_aux  = (*coke_tracker->image_frame).clone();
    ros::Time stamps_ros= *coke_tracker->stamps_ros;

    prepare_image(image_frame_aux,coke_tracker->image_rgb,coke_tracker->frame_count,\
      coke_tracker->image_gray,coke_tracker->cameraMatrix,coke_tracker->distCoeffs,\
      coke_tracker->fx,coke_tracker->fy,coke_tracker->cx,coke_tracker->cy,coke_tracker->reduction);

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),"bgr8",coke_tracker->image_gray).toImageMsg();
  //  pub_image->publish(msg);
}

void prepare_image(cv::Mat &image_frame, cv::Mat &image_rgb,int &frame_count,cv::Mat &image_gray,cv::Mat cameraMatrix,cv::Mat distCoeffs,double &fx,double &fy, double &cx,double &cy, int reduction)
{
if (image_frame.type()==CV_8UC1)
   {
          //input image is grayscale
          cv::cvtColor(image_frame, image_frame, CV_GRAY2RGB);//If gray convert to color.
   }
   cv::resize(image_frame,image_frame,cv::Size(image_frame.cols/reduction,image_frame.rows/reduction),0,0,cv::INTER_LINEAR);//resize based on reduction ratio


   cv::Mat image_ff = image_frame.clone();//make a copy of the image frame

   cv::Size ksize,ksize1;//save the rows and coloumns
   ksize.width = image_frame.cols;
   ksize.height = image_frame.rows;

   cv::Mat newCameraMatrix;

   cv::Mat cameraMatrixAux = cameraMatrix.clone();//camera Matrix read from data and copied at the constructor
   cameraMatrixAux/=reduction;// new CameraMatrix after reduction
   cameraMatrixAux.at<double>(2,2)=1;//except the last element in the matrix

   double alpha = 0;//Free scale param. Necessary for next step
   newCameraMatrix = cv::getOptimalNewCameraMatrix(cameraMatrixAux,distCoeffs,ksize,alpha,ksize1);//Take Distortion coefficient into account.


   cv::undistort(image_frame,image_ff,cameraMatrixAux,distCoeffs,newCameraMatrix);//get the undistorted image after Camera Calib

   image_frame = image_ff.clone();//image_ff only used to temporarily store the undistorted image

   //Focal Lengths and Camera Centers 
   fx = newCameraMatrix.at<double>(0,0);
   fy = newCameraMatrix.at<double>(1,1);
   cx = newCameraMatrix.at<double>(0,2);
   cy = newCameraMatrix.at<double>(1,2);
   image_rgb = image_frame.clone();//image_rgb now contains the undistorted image
   frame_count++;//increase the image_frame count
   cv::cvtColor(image_rgb,image_gray,CV_RGB2GRAY);//convert it back to gray and save it in image_to_track
   image_gray.convertTo(image_gray, CV_64FC1);//convert it into 64bits floating point data.//GUESS:63 bits,double,gray image,1 channel
   image_gray /= (255*1.0);//why?
    //image_gray must have full data.and image_to_track will have scaled data.But how does that affect?
}
