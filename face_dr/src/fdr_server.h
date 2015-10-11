/*

Copyright 2015 Alaa El Khatib.

    This file is part of the face_dr ROS package.

    face_dr is free software: you can redistribute it and/or modify
    it under the terms of the Lesser GNU General Public License as published by
    the Free Software Foundation, version 3.

    face_dr is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    Lesser GNU General Public License for more details.

    You should have received a copy of the Lesser GNU General Public License
    along with face_dr.  If not, see <http://www.gnu.org/licenses/>.


Face detection and recognition ROS actionlib server.
Subscribes to: an /usb_cam/image_raw through an ImageTransport class.

Author: Alaa El Khatib
Last updated: 08.10.2015

The code below is inspired by, builds upon, and/or uses code from the following source(s):

   -face_recognition ROS package by Pouyan Ziafati, shared under a Creative Commons Attribution-NonCommercial 3.0 Unported license (http://creativecommons.org/licenses/by-nc/3.0/).

   -OpenCV's FaceReconizer tutorial (http://docs.opencv.org/modules/contrib/doc/facerec/facerec_tutorial.html) code by Philipp Wagner, shared under a BSD Simplified license (http://www.opensource.org/licenses/bsd-license). The license terms are reproduced below.
   [*
    * Copyright (c) 2011. Philipp Wagner <bytefish[at]gmx[dot]de>.
    * Released to public domain under terms of the BSD Simplified license.
    *
    * Redistribution and use in source and binary forms, with or without
    * modification, are permitted provided that the following conditions are met:
    *   * Redistributions of source code must retain the above copyright
    *     notice, this list of conditions and the following disclaimer.
    *   * Redistributions in binary form must reproduce the above copyright
    *     notice, this list of conditions and the following disclaimer in the
    *     documentation and/or other materials provided with the distribution.
    *   * Neither the name of the organization nor the names of its contributors
    *     may be used to endorse or promote products derived from this software
    *     without specific prior written permission.
    *]

   -OpenCV's Histogram Calculation tutorial (http://docs.opencv.org/doc/tutorials/imgproc/histograms/histogram_calculation/histogram_calculation.html).

*/


#ifndef FDR_SERVER_H
#define FDR_SERVER_H

#include <vector>
#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "image_transport/image_transport.h"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "face_dr/fdrAction.h"
#include "sensor_msgs/image_encodings.h"

#include <sstream>
#include "cv_bridge/cv_bridge.h"

typedef actionlib::SimpleActionServer<face_dr::fdrAction> FDR_SERVER;
typedef face_dr::fdrFeedback FDR_FEEDBACK;
typedef face_dr::fdrResult FDR_RESULT;
typedef face_dr::fdrGoalConstPtr FDR_GOAL_Ptr;

std::string CSV_FILE = "/home/alaa/ros_data/face_images/faces_csv.txt";
std::string FACE_IMAGES_FILE = "/home/alaa/ros_data/face_images/";
std::string UB_IMAGES_FILE = "/home/alaa/ros_data/upper_body/";
std::string HAAR_CASCADE_FILE = "/home/alaa/ros_data/haarcascade_frontalface_alt.xml";
std::string LBP_CASCADE_FILE = "/home/alaa/ros_data/lbpcascade_frontalface.xml";
std::string UB_CASCADE_FILE = "/home/alaa/ros_data/haarcascade_upperbody.xml";
std::string PCA_MODEL_FILE = "/home/alaa/ros_data/face_images/pca_model.yml";
std::string LABEL_TO_NAME_FILE = "/home/alaa/ros_data/face_images/label_to_name.txt";
int PCA_DIM = 30;
int PCA_THRSH = 8000;

class FDRServer
{
   
   ros::NodeHandle nh;
   FDR_SERVER fdr_as;
   image_transport::ImageTransport img_transp;
   image_transport::Subscriber img_sub;
   FDR_FEEDBACK fdr_feedback;
   FDR_RESULT fdr_result;
   FDR_GOAL_Ptr fdr_goal;
   int counter_1;
   cv::CascadeClassifier face_cascade_1;
   cv::CascadeClassifier face_cascade_2;
   cv::CascadeClassifier face_cascade;
   cv::CascadeClassifier ub_cascade;
   std::string DETECT_m1;
   std::string DETECT_m2;
   std::string TRAIN_m1;
   std::string RECOGNIZE_m1;
   std::string RECOGNIZE_m2;

public:
   
   FDRServer(std::string);
   void goalCB();
   void preemptCB();
   void procCB(const sensor_msgs::ImageConstPtr&);
   
};

std::vector<cv::Rect> detect_faces_cascade(const cv::Mat&, cv::CascadeClassifier);

void read_csv(const std::string&, std::vector<cv::Mat>&, std::vector<int>&, std::vector<std::string>&, char separator = ';');

std::vector<cv::Rect> detect_ubs_cascade(const cv::Mat&, cv::CascadeClassifier);

void calc_hist(const cv::Mat&, cv::Mat&);

#endif
