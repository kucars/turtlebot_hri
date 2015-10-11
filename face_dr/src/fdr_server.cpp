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


#include "fdr_server.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <boost/filesystem.hpp>
#include <fstream>
#include "opencv2/contrib/contrib.hpp"

struct Prediction
{
   std::string name;
   double confidence;
};

FDRServer::FDRServer(std::string as_name) : fdr_as(nh, as_name, false), img_transp(nh)
{
   fdr_as.registerGoalCallback(boost::bind(&FDRServer::goalCB, this));
   fdr_as.registerPreemptCallback(boost::bind(&FDRServer::preemptCB, this));
   img_sub = img_transp.subscribe("/usb_cam/image_raw", 1, &FDRServer::procCB, this);
   DETECT_m1 = "DETECT_HAAR_CASCADE";
   DETECT_m2 = "DETECT_LBP_CASCADE";
   TRAIN_m1 = "TRAIN_PCA";
   RECOGNIZE_m1 = "RECOGNIZE_PCA_HAAR";
   RECOGNIZE_m2 = "RECOGNIZE_PCA_LBP";
   fdr_as.start();      
   if (!face_cascade_1.load(HAAR_CASCADE_FILE))
   {
      ROS_INFO("error loading Haar cascade classifier\nserver node is shuting down...");
      ros::shutdown();
   }

   if (!face_cascade_2.load(LBP_CASCADE_FILE))
   {
      ROS_INFO("error loading LBP cascade classifier\nserver node is shuting down...");
      ros::shutdown();
   }

   if (!ub_cascade.load(UB_CASCADE_FILE))
   {
      ROS_INFO("error loading UB cascade classifier\nserver node is shuting down...");
      ros::shutdown();
   }
}

void FDRServer::goalCB()
{
   fdr_goal = fdr_as.acceptNewGoal();
   counter_1 = 0;
}   

void FDRServer::preemptCB()
{
   ROS_INFO("Goal is preempted");
   fdr_as.setPreempted();
}

void FDRServer::procCB(const sensor_msgs::ImageConstPtr& msg)
{
   if (!fdr_as.isActive())
      return;

   if (std::strcmp((fdr_goal->order).c_str(), DETECT_m1.c_str())==0 || std::strcmp((fdr_goal->order).c_str(), DETECT_m2.c_str())==0)
   {  
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
      cv::Mat img_msg = cv_ptr->image;

      if (std::strcmp((fdr_goal->order).c_str(), DETECT_m1.c_str())==0)
         face_cascade = face_cascade_1;
      else if (std::strcmp((fdr_goal->order).c_str(), DETECT_m2.c_str())==0)
         face_cascade = face_cascade_2;

      std::vector<cv::Rect> detected_faces = detect_faces_cascade(img_msg, face_cascade);
      if (detected_faces.size()!=1) return;
      else
      {
         ros::Rate CBrate(2.0);
         counter_1++;
         std::ostringstream save_name;
         save_name << FACE_IMAGES_FILE << (fdr_goal->name).c_str();
         struct stat path_chk;
         if (!(stat(save_name.str().c_str(), &path_chk)==0 && S_ISDIR(path_chk.st_mode)))
            boost::filesystem::create_directories(save_name.str());
         save_name << "/" << (fdr_goal->name).c_str() << "_" << counter_1 << ".jpg";
         cv::Mat img_roi = img_msg(detected_faces[0]);
         cv::Mat img_gray;
         cvtColor(img_roi, img_gray, cv::COLOR_BGR2GRAY);
         cv::Mat img_resized;
         cv::resize(img_gray, img_resized, cv::Size(256, 256));
         cv::imwrite(save_name.str(), img_resized);
         
         std::ofstream csv_ost(CSV_FILE.c_str(), std::ios_base::app);
         csv_ost << save_name.str().c_str() << ";" << (fdr_goal->name).c_str() << std::endl;
         csv_ost.close();

         ROS_INFO("%d image(s) added so far", counter_1);
         if (counter_1==(fdr_goal->num_of_samples))
         {
            ROS_INFO("finished saving %d face images for %s.", (fdr_goal->num_of_samples), (fdr_goal->name).c_str());
            fdr_result.name = fdr_goal->name;
            fdr_result.order = fdr_goal->order;
            fdr_result.num_of_samples = fdr_goal->num_of_samples;
            fdr_as.setSucceeded(fdr_result);
         }
         CBrate.sleep();
      }
         
   }

  
   else if (std::strcmp((fdr_goal->order).c_str(), TRAIN_m1.c_str())==0)
   {
      ROS_INFO("training...");
      std::vector<cv::Mat> face_images_vector;
      std::vector<int> class_labels;
      std::vector<std::string> label_to_name;
      read_csv(CSV_FILE, face_images_vector, class_labels, label_to_name);
      cv::Ptr<cv::FaceRecognizer> pca_model = cv::createEigenFaceRecognizer(PCA_DIM, PCA_THRSH);
      pca_model->train(face_images_vector, class_labels);
      pca_model->save(PCA_MODEL_FILE);
      std::ofstream ost(LABEL_TO_NAME_FILE.c_str(), std::ios_base::out);
      for (int i = 0; i < label_to_name.size(); i++)
         ost << label_to_name[i] << std::endl;
      ost.close();
      ROS_INFO("Training done. Classifier model saved to %s", PCA_MODEL_FILE.c_str());
      fdr_result.name = fdr_goal->name;
      fdr_result.order = fdr_goal->order;
      fdr_result.num_of_samples = fdr_goal->num_of_samples;
      fdr_as.setSucceeded(fdr_result);
   }

   else if (std::strcmp((fdr_goal->order).c_str(), RECOGNIZE_m1.c_str())==0 || std::strcmp((fdr_goal->order).c_str(), RECOGNIZE_m2.c_str())==0)
   {
      ros::Rate CBrate(4.0);
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
      cv::Mat img_msg = cv_ptr->image;


      if (std::strcmp((fdr_goal->order).c_str(), RECOGNIZE_m1.c_str())==0)
         face_cascade = face_cascade_1;
      else if (std::strcmp((fdr_goal->order).c_str(), RECOGNIZE_m2.c_str())==0)
         face_cascade = face_cascade_2;

      std::vector<cv::Rect> detected_faces = detect_faces_cascade(img_msg, face_cascade);
      if (detected_faces.size()>0)
      {
         bool hist_exists = false;
         bool detected_is_known = false;
         cv::Mat hue_hist, ub_img;
         if (detected_faces.size()==1)
         {
            std::vector<cv::Rect> ub_rects = detect_ubs_cascade(img_msg, ub_cascade);
            if (ub_rects.size()==1)
            {
               double nx = ub_rects[0].x+ub_rects[0].width*0.3;
               double ny = ub_rects[0].y+ub_rects[0].height*0.7;
               double nwidth = ub_rects[0].width*0.4;
               double nheight = ub_rects[0].height*0.3;
               cv::Rect n_ub(nx, ny, nwidth, nheight);
               ub_img = img_msg(n_ub);
               calc_hist(ub_img, hue_hist);
               hist_exists = true;
            }
         }
         std::ifstream ist(LABEL_TO_NAME_FILE.c_str(), std::ios::in);
         std::string tmp_name;
         std::vector<std::string> label_to_name;
         while (ist >> tmp_name)
            label_to_name.push_back(tmp_name);
         ist.close();
         int predicted_label;
         double confidence;
         std::vector<Prediction> predictions;
         for (int i = 0; i < detected_faces.size(); i++)
         {
            confidence = 0;
            predicted_label = -1;
            cv::Ptr<cv::FaceRecognizer> pca_model = cv::createEigenFaceRecognizer(PCA_DIM, PCA_THRSH);
            pca_model->load(PCA_MODEL_FILE);
            cv::Mat img_roi = img_msg(detected_faces[i]);
            cv::Mat img_gray;
            cvtColor(img_roi, img_gray, CV_BGR2GRAY);
            cv::Mat img_resized;
            cv::resize(img_gray, img_resized, cv::Size(256, 256));
            pca_model->predict(img_resized, predicted_label, confidence);
            std::string predicted_name = (predicted_label > -1 ? label_to_name[predicted_label] : "a stranger");
            if (predicted_label > -1) detected_is_known = true; 
            Prediction tmp = {predicted_name, confidence};
            predictions.push_back(tmp);
            ROS_INFO("%s was recognized with confidence %f", tmp.name.c_str(),  predicted_label > -1 ? tmp.confidence : PCA_THRSH);
            fdr_feedback.order = fdr_goal->order;
            fdr_feedback.name = tmp.name;
            fdr_feedback.confidence = tmp.confidence;
            fdr_as.publishFeedback(fdr_feedback);
         }
         if (hist_exists && detected_is_known)
         {
            int count;
            std::ostringstream dc_file_name;
            dc_file_name << UB_IMAGES_FILE << predictions[0].name;
            struct stat path_chk;
            if (!(stat(dc_file_name.str().c_str(), &path_chk)==0 && S_ISDIR(path_chk.st_mode)))
               boost::filesystem::create_directories(dc_file_name.str());
            std::ostringstream dc_yaml_name;
            dc_yaml_name << dc_file_name.str() << "/dc_log.yml";

            std::ostringstream dc_meta_name;
            dc_meta_name << dc_file_name.str() << "/dc_meta_data.yml";

            if (!(stat(dc_meta_name.str().c_str(), &path_chk)==0))
               count = 1;
            else
            {
               cv::FileStorage fsm(dc_meta_name.str(), cv::FileStorage::READ);
               count = (int)fsm["count"];
               fsm.release();
               count++;
            }
            if (count < 31)
            {
               std::string node_name = "Hue Histogram "+std::to_string(count);
               std::string t_node_name = "Time "+std::to_string(count);
               cv::FileStorage fs(dc_yaml_name.str(), cv::FileStorage::APPEND);
               std::time_t rawtime; std::time(&rawtime);
               fs << t_node_name << std::asctime(std::localtime(&rawtime));
               fs << node_name << hue_hist;
               fs.release();
               cv::FileStorage fsm(dc_meta_name.str(), cv::FileStorage::WRITE);
               fsm << "count" << count;
               fsm.release();
               std::ostringstream dc_img_name;
               dc_img_name << dc_file_name.str() << "/dc_img.jpg";
               cv::imwrite(dc_img_name.str(), ub_img);
            }
         }
      }
      else
      {
         //IF NO FACE IS DETECTED AND UB IS DETECTED
         //CLASSIFY COLOR
         fdr_feedback.order = fdr_goal->order;
         fdr_feedback.name = "none";
         fdr_feedback.confidence = 0.0;
         fdr_as.publishFeedback(fdr_feedback);
      }
      CBrate.sleep();
   }



   else 
   {
      ROS_INFO("Unrecognized order. Use one of the following: 'DETECT_HAAR_CASCADES' or 'TRAIN_PCA' or 'RECOGNIZE_PCA'.");
      ROS_INFO("%s",(fdr_goal->order).c_str());
      preemptCB();
   }
}

//**************************//
std::vector<cv::Rect> detect_faces_cascade(const cv::Mat& frame, cv::CascadeClassifier face_cascade)
{
    std::vector<cv::Rect> faces;
    cv::Mat frame_gray;

    cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);
    equalizeHist(frame_gray, frame_gray);

    face_cascade.detectMultiScale(frame_gray, faces, 1.1, 5, 0, cv::Size(64, 64));

    return faces;
}


//**************************//
void read_csv(const std::string& filename, std::vector<cv::Mat>& face_images_vector, std::vector<int>& class_labels, std::vector<std::string>& label_to_name, char separator)
{
   int current_label = -1;
   std::string prev_name = "!!isAnImpossibleName!!";
   std::ifstream file(filename.c_str());
   std::string line, path, person_name;
   while (getline(file, line))
   {
       std::stringstream liness(line);
       getline(liness, path, separator);
       getline(liness, person_name);
       if(!path.empty() && !person_name.empty())
       {
          face_images_vector.push_back(cv::imread(path, -1));
          if (person_name!=prev_name)
          {
             current_label++;
             prev_name = person_name;
             label_to_name.push_back(person_name);
          }
          class_labels.push_back(current_label);
       }
   }
}


//**************************//
std::vector<cv::Rect> detect_ubs_cascade(const cv::Mat& frame, cv::CascadeClassifier ub_cascade)
{
   std::vector<cv::Rect> ubs;
   ub_cascade.detectMultiScale(frame, ubs, 1.1, 3, 0, cv::Size(256, 256));
   return ubs;
}



//*************************//
void calc_hist(const cv::Mat& ub_img, cv::Mat& hue_hist)
{
   cv::Mat hsv_img;
   cvtColor(ub_img, hsv_img, CV_BGR2HSV);
   std::vector<cv::Mat> hsv_planes;
   cv::split( hsv_img, hsv_planes );

   int histSize = 256;

   float range[] = { 0, 256 } ;
   const float* histRange = { range };

   bool uniform = true; bool accumulate = false;

   cv::calcHist( &hsv_planes[0], 1, 0, cv::Mat(), hue_hist, 1, &histSize, &histRange, uniform, accumulate );

   int hist_w = 512; int hist_h = 400;
   int bin_w = cvRound( (double) hist_w/histSize );

   cv::Mat histImage( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );

   cv::normalize(hue_hist, hue_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );
}
