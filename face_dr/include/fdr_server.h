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

#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "image_transport/image_transport.h"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "face_dr/fdrAction.h"
#include "sensor_msgs/image_encodings.h"
#include "opencv2/ml/ml.hpp"
#include "cv_bridge/cv_bridge.h"

typedef actionlib::SimpleActionServer<face_dr::fdrAction> FDR_SERVER;
typedef face_dr::fdrFeedback FDR_FEEDBACK;
typedef face_dr::fdrResult FDR_RESULT;
typedef face_dr::fdrGoalConstPtr FDR_GOAL_Ptr;

//define paths to folders
std::string ROS_DATA = "/home/alaa/ros_data/";
std::string FACE_FOLDER = ROS_DATA + "faces/";
std::string FACE_IMGS_FOLDER = FACE_FOLDER + "face_images/";
std::string U_BODY_FOLDER = ROS_DATA + "upper_body/";
std::string HUE_HISTS_FOLDER = U_BODY_FOLDER + "hue_hists/";
std::string DRESS_COLOR_IMGS_FOLDER = U_BODY_FOLDER + "dc_images/";

//define paths to files
std::string FACE_CSV = FACE_FOLDER + "faces_csv.txt";
std::string FACE_HAAR_DETECTOR = FACE_FOLDER + "haarcascade_frontalface_alt.xml";
std::string FACE_LBP_DETECTOR = FACE_FOLDER + "lbpcascade_frontalface.xml";
std::string FACE_PCA_RECOGNIZER = FACE_FOLDER + "pca_model.yml";
std::string FACE_NAMES = FACE_FOLDER + "class_names.txt";

std::string U_BODY_HAAR_DETECTOR = U_BODY_FOLDER + "haarcascade_upperbody.xml";
std::string COLOR_CLASSIFIER = U_BODY_FOLDER + "color_classifier.yml";
std::string U_BODY_NAMES = U_BODY_FOLDER + "class_names.txt";

//define face recognizer parameters
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
   cv::CascadeClassifier face_cascade_1; //loads Haar
   cv::CascadeClassifier face_cascade_2; //laods LBP
   cv::CascadeClassifier face_cascade; //uses either Haar or LBP
   cv::CascadeClassifier ub_cascade; //loads Haar


public:
   
   FDRServer(std::string);
   void goalCB();
   void preemptCB();
   void procCB(const sensor_msgs::ImageConstPtr&);
   
};

#endif