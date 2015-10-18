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
#include "fdr_utilities.h"
#include <boost/filesystem.hpp>
#include <fstream>
#include "opencv2/contrib/contrib.hpp"


FDRServer::FDRServer(std::string as_name) : fdr_as(nh, as_name, false), img_transp(nh)
{
   fdr_as.registerGoalCallback(boost::bind(&FDRServer::goalCB, this));
   fdr_as.registerPreemptCallback(boost::bind(&FDRServer::preemptCB, this));
   img_sub = img_transp.subscribe("/usb_cam/image_raw", 1, &FDRServer::procCB, this);

   fdr_as.start();      
   if (!face_cascade_1.load(FACE_HAAR_DETECTOR))
   {
      ROS_INFO("error loading Haar cascade classifier\nserver node is shuting down...");
      ros::shutdown();
   }

   if (!face_cascade_2.load(FACE_LBP_DETECTOR))
   {
      ROS_INFO("error loading LBP cascade classifier\nserver node is shuting down...");
      ros::shutdown();
   }

   if (!ub_cascade.load(U_BODY_HAAR_DETECTOR))
   {
      ROS_INFO("error loading UB cascade classifier\nserver node is shuting down...");
      ros::shutdown();
   }
}

void FDRServer::goalCB()
{
   fdr_goal = fdr_as.acceptNewGoal();
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

   if (fdr_goal->order == "DETECT_FACES_HAAR" ||
	   fdr_goal->order == "DETECT_FACES_LBP")
   {  
	  
	  cv::Mat img_msg = cv_bridge::toCvCopy(msg, "bgr8")->image;

	  if (fdr_goal->order == "DETECT_FACES_HAAR")
		  face_cascade = face_cascade_1;
	  else if (fdr_goal->order == "DETECT_FACES_LBP")
		  face_cascade = face_cascade_2;

      std::vector<cv::Rect> detected_faces = detect_faces_cascade(img_msg, face_cascade);

      if (detected_faces.size() == 1)
      {
         ros::Rate CBrate(2.0);
	 int dir_count = count_files(FACE_IMGS_FOLDER + fdr_goal->name);

         if (dir_count < (fdr_goal->num_of_samples))
         {
            cv::Mat img_roi = img_msg(detected_faces[0]);
            cv::Mat img_gray;
            cvtColor(img_roi, img_gray, CV_BGR2GRAY);
            cv::Mat img_resized;
            cv::resize(img_gray, img_resized, cv::Size(256, 256));
	    save_image(img_resized, FACE_IMGS_FOLDER, fdr_goal->name, dir_count + 1);
            append_to_csv(FACE_CSV, FACE_IMGS_FOLDER, fdr_goal->name, dir_count + 1);
            ROS_INFO("%d image(s) added so far", (dir_count + 1));
            fdr_feedback.order = fdr_goal->order;
            fdr_feedback.name = fdr_goal->name;
            fdr_feedback.confidence = dir_count + 1;
            fdr_feedback.time_first = "";
            fdr_feedback.time_last = "";
            fdr_as.publishFeedback(fdr_feedback);
         }
         else
         {
            ROS_INFO("finished saving %d face images for %s.", (fdr_goal->num_of_samples), (fdr_goal->name).c_str());
            fdr_result.name = fdr_goal->name;
            fdr_result.order = fdr_goal->order;
            fdr_result.num_of_samples = fdr_goal->num_of_samples;
            fdr_as.setSucceeded(fdr_result);
         }
         CBrate.sleep();
      }
      else return;
   }

  
   else if (fdr_goal->order == "TRAIN_PCA")
   {
      ROS_INFO("training...");
      std::vector<cv::Mat> face_images_vector;
      std::vector<int> class_labels;
      std::vector<std::string> label_to_name;
      read_csv(FACE_CSV, face_images_vector, class_labels, label_to_name, ';');
      cv::Ptr<cv::FaceRecognizer> pca_model = cv::createEigenFaceRecognizer(PCA_DIM, PCA_THRSH);
      pca_model->train(face_images_vector, class_labels);
      pca_model->save(FACE_PCA_RECOGNIZER);
      write_vector(FACE_NAMES, label_to_name);
      ROS_INFO("Training done. Classifier model saved to %s", FACE_PCA_RECOGNIZER.c_str());
      fdr_result.name = fdr_goal->name;
      fdr_result.order = fdr_goal->order;
      fdr_result.num_of_samples = fdr_goal->num_of_samples;
      fdr_as.setSucceeded(fdr_result);
   }

   else if (fdr_goal->order == "RECOGNIZE_PCA_HAAR" ||
      fdr_goal->order == "RECOGNIZE_PCA_LBP")
   {
      ros::Rate CBrate(4.0);
      cv::Mat img_msg = cv_bridge::toCvCopy(msg, "bgr8")->image;

      if (fdr_goal->order == "RECOGNIZE_PCA_HAAR")
         face_cascade = face_cascade_1;
      else if (fdr_goal->order == "RECOGNIZE_PCA_LBP")
         face_cascade = face_cascade_2;

      std::vector<cv::Rect> detected_faces = detect_faces_cascade(img_msg, face_cascade);
      if (detected_faces.size()>0)
      {
         bool hist_exists = false;
         bool detected_is_known = false;
         cv::Mat hue_hist, dc_img;
         if (detected_faces.size()==1)
         {
	    std::vector<cv::Rect> ub_rects;
	    ub_cascade.detectMultiScale(img_msg, ub_rects, 1.1, 3, 0, cv::Size(256, 256));
            if (ub_rects.size()==1)
            {
	       dc_img = extract_dress_color(ub_rects[0], img_msg);
               calc_hist(dc_img, hue_hist);
               hist_exists = true;
            }
         }
	 std::vector<std::string> label_to_name;
	 read_vector(FACE_NAMES, label_to_name);
	 std::string predicted_name;
         for (int i = 0; i < detected_faces.size(); i++)
         {
            double confidence = 0;
            int predicted_label = -1;
            cv::Ptr<cv::FaceRecognizer> pca_model = cv::createEigenFaceRecognizer(PCA_DIM, PCA_THRSH);
            pca_model->load(FACE_PCA_RECOGNIZER);
            cv::Mat img_roi = img_msg(detected_faces[i]);
            cv::Mat img_gray;
            cvtColor(img_roi, img_gray, CV_BGR2GRAY);
            cv::Mat img_resized;
            cv::resize(img_gray, img_resized, cv::Size(256, 256));
            pca_model->predict(img_resized, predicted_label, confidence);
            predicted_name = (predicted_label > -1 ? label_to_name[predicted_label] : "a stranger");
            if (predicted_label > -1) detected_is_known = true;
            ROS_INFO("%s was recognized", predicted_name.c_str());
            fdr_feedback.order = fdr_goal->order;
            fdr_feedback.name = predicted_name;
            fdr_feedback.confidence = confidence;
            fdr_feedback.time_first = "";
            fdr_feedback.time_last = "";
            fdr_as.publishFeedback(fdr_feedback);
         }
         if (hist_exists && detected_is_known) //hist_exists is true iff only 1 face is detected, ergo detected_is_known refers to the same face despite the loop
         {
            int dir_count = count_files(DRESS_COLOR_IMGS_FOLDER + predicted_name);
            if (dir_count < 30)
	    {
               append_to_yaml(HUE_HISTS_FOLDER, predicted_name, hue_hist, dir_count + 1);
	       save_image(dc_img, DRESS_COLOR_IMGS_FOLDER, predicted_name, dir_count + 1);
	       write_yaml(U_BODY_FOLDER, "dc_data_status.yml", "status", "CHANGED");
	    }
         }
      }
      else
      {
         std::vector<cv::Rect> ub_rects;
	 ub_cascade.detectMultiScale(img_msg, ub_rects, 1.1, 3, 0, cv::Size(256, 256));
	 if (ub_rects.size()>0)
         {
            if (boost::filesystem::is_empty(DRESS_COLOR_IMGS_FOLDER))
            {
               ROS_INFO("%lu unidentified person(s) detected using upper body information", ub_rects.size());
               fdr_feedback.order = fdr_goal->order;
               fdr_feedback.name = "multiple UBs";
               fdr_feedback.confidence = ub_rects.size();
               fdr_feedback.time_first = "";
               fdr_feedback.time_last = "";
               fdr_as.publishFeedback(fdr_feedback);
            }
            else
            {
               bool classifier_exists = false;
	       std::string classifier_status;
	       read_yaml<std::string>(U_BODY_FOLDER, "dc_data_status.yml", "status", classifier_status);
               CvDTree dTree;
               CvDTreeParams dTreeParams;
               dTreeParams.min_sample_count = 4;
               dTreeParams.cv_folds = 2;
               std::vector<std::string> label_to_ub_name;
               if (!(classifier_status == "UPDATED" &&
	    	     boost::filesystem::exists(boost::filesystem::path(COLOR_CLASSIFIER))))
               {  
                  cv::Mat trainingData;
                  cv::Mat responseData;
                  read_ub_yml(HUE_HISTS_FOLDER, DRESS_COLOR_IMGS_FOLDER,trainingData, responseData, label_to_ub_name); 
                  if (trainingData.size().width > 20)
                  { 
                     dTree.train(trainingData, CV_COL_SAMPLE, responseData, cv::Mat(), cv::Mat(), cv::Mat(), cv::Mat(), dTreeParams);
                     dTree.save(COLOR_CLASSIFIER.c_str());
		     write_yaml(U_BODY_FOLDER, "dc_data_status.yml", "status", "UPDATED");
		     write_vector(U_BODY_NAMES, label_to_ub_name);
                     classifier_exists = true;
                  }
                  
               }
               else
               {
                  dTree.load(COLOR_CLASSIFIER.c_str());
		  read_vector(U_BODY_NAMES, label_to_ub_name);
                  classifier_exists = true;
               }

               if (classifier_exists)
               {
                  cv::Mat hue_hist, dc_img;
                  for (int i = 0; i < ub_rects.size(); i++)
                  {
		     dc_img = extract_dress_color(ub_rects[i], img_msg);
                     calc_hist(dc_img, hue_hist);
                     int ub_prediction = dTree.predict(hue_hist.t())->value;
                     if (ub_prediction > -1)
                     {
                        std::string ub_name = label_to_ub_name[ub_prediction];
			int dir_count = count_files(DRESS_COLOR_IMGS_FOLDER + ub_name);
			std::string t1, t2;
			read_yaml<std::string>(HUE_HISTS_FOLDER + ub_name, "/dc_log.yml", "Time 1", t1);
			read_yaml<std::string>(HUE_HISTS_FOLDER + ub_name, "/dc_log.yml", "Time " + std::to_string(dir_count), t2);
                        ROS_INFO("%s was recognized using the color of his/her shirt, based on samples accumulated between %s and %s", ub_name.c_str(), t1.c_str(), t2.c_str());
                        fdr_feedback.order = fdr_goal->order;
                        fdr_feedback.name = ub_name;
                        fdr_feedback.confidence = -1;
                        fdr_feedback.time_first = t1;
                        fdr_feedback.time_last = t2;
                        fdr_as.publishFeedback(fdr_feedback);
                     }
                  }
               }
               else 
               {
                  ROS_INFO("%lu unidentified person(s) detected using upper body information", ub_rects.size());
                  fdr_feedback.order = fdr_goal->order;
                  fdr_feedback.name = "multiple UBs";
                  fdr_feedback.confidence = ub_rects.size();
                  fdr_as.publishFeedback(fdr_feedback);
               }
               
            }
         }
         else
         {
            fdr_feedback.order = fdr_goal->order;
            fdr_feedback.name = "none";
            fdr_feedback.confidence = 0.0;
            fdr_as.publishFeedback(fdr_feedback);
         }
      }
      CBrate.sleep();
   }



   else 
   {      
      ROS_INFO("%s",(fdr_goal->order).c_str());
      ROS_INFO("Unrecognized order. Use one of the following: 'DETECT_FACES_HAAR' or 'DETECT_FACES_LBP' or 'TRAIN_PCA' or 'RECOGNIZE_PCA_HAAR' or 'RECOGNIZE_PCA_LBP'.");
      preemptCB();
   }
}

