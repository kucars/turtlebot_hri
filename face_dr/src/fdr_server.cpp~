/*

Copyright 2015.

NOTICE: THIS FILE USES OPENCV'S NONFREE LIBRARIES (SURF) WHICH COULD HAVE A DIFFERENT LICENSE. CHECK BEFORE YOU USE!

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


Author: Alaa El Khatib
Last updated: 16.11.2015

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
-"Real Time Object Recognition using SURF" by Frank (https://github.com/doczhivago/rtObjectRecognition),
	which relies on OpenCV's tutorial (http://docs.opencv.org/2.4/doc/tutorials/features2d/feature_homography/feature_homography.html).
*/


#include "fdr_server.h"
#include "fdr_utilities.h"
#include <boost/filesystem.hpp>
#include <fstream>
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/calib3d/calib3d.hpp"

FDRServer::FDRServer(std::string as_name, std::string cam_name, std::string standalone) :
fdr_as(nh, as_name, false),
img_transp(nh),
rgb_sub(img_transp, cam_name + "/rgb/image_color", 1, image_transport::TransportHints("compressed")),
depth_sub(img_transp, cam_name + "/depth_registered/image_raw", 1),
sync(img_sync_policy(10), rgb_sub, depth_sub)
{

	fdr_as.registerGoalCallback(boost::bind(&FDRServer::goalCB, this));
	fdr_as.registerPreemptCallback(boost::bind(&FDRServer::preemptCB, this));
	sync.registerCallback(boost::bind(&FDRServer::face_cb, this, _1, _2));


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

	if (standalone == "true")
		STANDALONE = true;
	else
		STANDALONE = false;

	//CHANGE THIS TO REFLECT YOUR MANUALLY CALCULATED TRANSFORM
	if (STANDALONE)
	{
		tf::Transform transform_tmp;
		transform_tmp.setOrigin(tf::Vector3(-3.2, -0.12, 0.0));
		tf::Quaternion q;
		q.setRPY(0, 0, 0.84);
		transform_tmp.setRotation(q);
		transform_sa = transform_tmp;
	}
	cv::namedWindow("Object detection", CV_WINDOW_AUTOSIZE);
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

void FDRServer::face_cb(const sensor_msgs::ImageConstPtr& rgb_img_ptr, const sensor_msgs::ImageConstPtr& depth_img_ptr)
{

	if (!fdr_as.isActive())
		return;

	if (fdr_goal->order == "DETECT_FACES_HAAR" ||
		fdr_goal->order == "DETECT_FACES_LBP")
	{

		cv::Mat img_msg = cv_bridge::toCvCopy(rgb_img_ptr, "mono8")->image;

		if (fdr_goal->order == "DETECT_FACES_HAAR")
			face_cascade = face_cascade_1;
		else if (fdr_goal->order == "DETECT_FACES_LBP")
			face_cascade = face_cascade_2;
		std::vector<cv::Rect> detected_faces;
		face_cascade.detectMultiScale(img_msg, detected_faces, 1.1, 5, 0, cv::Size(64, 64));
		if (detected_faces.size() == 1)
		{
			ros::Rate CBrate(4.0);
			int dir_count = count_files(FACE_IMGS_FOLDER + fdr_goal->name);

			if (dir_count < (fdr_goal->num_of_samples))
			{
				cv::Mat img_roi = img_msg(detected_faces[0]);
				cv::Mat img_resized;
				cv::resize(img_roi, img_resized, cv::Size(256, 256));
				save_image(img_resized, FACE_IMGS_FOLDER, fdr_goal->name, dir_count + 1);
				append_to_csv(FACE_CSV, FACE_IMGS_FOLDER, fdr_goal->name, dir_count + 1);
				ROS_INFO("%d image(s) added so far", (dir_count + 1));
				fdr_feedback.order = fdr_goal->order;
				fdr_feedback.name = fdr_goal->name;
				fdr_feedback.result = "";
				fdr_feedback.xmidpt = dir_count + 1;
				fdr_feedback.depth = 0;
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
		fdr_goal->order == "RECOGNIZE_PCA_LBP" ||
		fdr_goal->order == "FIND_AND_FOLLOW_HAAR" ||
		fdr_goal->order == "FIND_AND_FOLLOW_LBP")
	{
		ros::Rate CBrate(2.0);
		ros::Time img_time = rgb_img_ptr->header.stamp;
		bool FIND_SUCCESS = false;
		double target_depth, target_midpt;
		cv::Mat img_bgr = cv_bridge::toCvCopy(rgb_img_ptr, "bgr8")->image;
		cv::Mat img_msg;
		cvtColor(img_bgr, img_msg, CV_BGR2GRAY);

		cv::Mat depth_msg = cv_bridge::toCvCopy(depth_img_ptr, "32FC1")->image;

		if (fdr_goal->order == "RECOGNIZE_PCA_HAAR" || fdr_goal->order == "FIND_AND_FOLLOW_HAAR")
			face_cascade = face_cascade_1;
		else if (fdr_goal->order == "RECOGNIZE_PCA_LBP" || fdr_goal->order == "FIND_AND_FOLLOW_LBP")
			face_cascade = face_cascade_2;


		std::vector<cv::Rect> detected_faces;
		face_cascade.detectMultiScale(img_msg, detected_faces, 1.1, 5, 0, cv::Size(64, 64));

		std::vector<cv::Rect> ub_rects;
		ub_cascade.detectMultiScale(img_msg, ub_rects, 1.1, 5, 0, cv::Size(256, 256));
		if (ub_rects.size() == 0 && detected_faces.size() == 0)
		{
			if (fdr_goal->order == "FIND_AND_FOLLOW_HAAR" ||
				fdr_goal->order == "FIND_AND_FOLLOW_LBP")
			{
				std::string name_ext = fdr_goal->name + "/last_seen.yml";
				boost::filesystem::path file_path(FACE_IMGS_FOLDER + name_ext);
				if (boost::filesystem::exists(file_path))
				{
					double time_seen;
					cv::Point2d loc_seen;
					read_yaml_2<double, cv::Point2d>(FACE_IMGS_FOLDER, name_ext, "time", time_seen, "location", loc_seen);
					fdr_feedback.order = fdr_goal->order;
					fdr_feedback.name = fdr_goal->name;
					fdr_feedback.result = "STORED";
					fdr_feedback.time_seen = time_seen;
					fdr_feedback.depth = loc_seen.x;
					fdr_feedback.xmidpt = loc_seen.y;
					fdr_as.publishFeedback(fdr_feedback);
				}
				else
				{
					fdr_feedback.order = fdr_goal->order;
					fdr_feedback.name = "none";
					fdr_feedback.result = "FAIL";
					fdr_as.publishFeedback(fdr_feedback);
				}

			}
			else
			{
				fdr_feedback.order = fdr_goal->order;
				fdr_feedback.name = "none";
				fdr_feedback.result = "FAIL";
				fdr_as.publishFeedback(fdr_feedback);
			}


		}

		else
		{
			std::vector<std::string> label_to_name;
			read_vector(FACE_NAMES, label_to_name);
			std::vector<std::string> predicted_names;
			std::vector<double> confidences;
			std::vector<double> face_depths;
			std::vector<double> face_midpts;
			std::vector<double> unknown_fdepths;
			std::vector<double> unknown_fmidpts;
			for (unsigned int i = 0; i < detected_faces.size(); i++)
			{
				double confidence = 0;
				int predicted_label = -1;
				cv::Ptr<cv::FaceRecognizer> pca_model = cv::createEigenFaceRecognizer(PCA_DIM, PCA_THRSH);
				pca_model->load(FACE_PCA_RECOGNIZER);
				cv::Mat img_roi = img_msg(detected_faces[i]);
				cv::Mat img_resized;
				cv::resize(img_roi, img_resized, cv::Size(256, 256));
				pca_model->predict(img_resized, predicted_label, confidence);


				cv::Mat face_depth = depth_msg(detected_faces[i]);
				double min_depth = 0;
				double x_real = 0;

				if (cv::countNonZero(face_depth) > 0.25 * face_depth.size().area())
				{
					cv::Point min_xy;
					cv::minMaxLoc(face_depth, &min_depth, NULL, &min_xy, NULL, face_depth != 0);
					min_depth /= 1000;
					x_real = -1 * (detected_faces[i].x + min_xy.x - (frame_width / 2))
						* (min_depth / focal_length);
				}

				if (predicted_label < 0)
				{
					unknown_fdepths.push_back(min_depth);
					unknown_fmidpts.push_back(x_real);
				}
				else
				{
					if (label_to_name[predicted_label] == fdr_goal->name)
					{
						FIND_SUCCESS = true;
						target_depth = min_depth;
						target_midpt = x_real;
					}
					predicted_names.push_back(label_to_name[predicted_label]);
					confidences.push_back(confidence);
					face_depths.push_back(min_depth);
					face_midpts.push_back(x_real);

				}
				if (fdr_goal->order == "RECOGNIZE_PCA_HAAR" ||
					fdr_goal->order == "RECOGNIZE_PCA_LBP")
				{
					fdr_feedback.order = fdr_goal->order;
					fdr_feedback.name = predicted_label < 0 ? "unknown" : label_to_name[predicted_label];
					fdr_feedback.result = "face";
					fdr_feedback.depth = min_depth;
					fdr_feedback.xmidpt = x_real;
					fdr_as.publishFeedback(fdr_feedback);
					if (predicted_label >= 0)
					{
						tf::Vector3 vec_tmp(min_depth + 1, x_real, 0.0);
						tf::Vector3 vec_sa = transform_sa(vec_tmp);
						cv::Point2d tpt(vec_sa.x(), vec_sa.y());
						std::string path_ext = label_to_name[predicted_label] + "/last_seen.yml";
						write_yaml_2<double, cv::Point2d>(FACE_IMGS_FOLDER, path_ext, "time", ros::WallTime::now().toSec(), "location", tpt);
					}

				}

			}

			bool classifier_exists = false;
			std::string classifier_status;
			if (!boost::filesystem::is_empty(DRESS_COLOR_IMGS_FOLDER))
			{
				if (!(classifier_status == "UPDATED" &&
					boost::filesystem::exists(boost::filesystem::path(COLOR_CLASSIFIER))))
				{
					cv::Mat trainingData;
					cv::Mat responseData;
					std::vector<std::string> label_to_ub_name;
					read_ub_yml(HUE_HISTS_FOLDER, DRESS_COLOR_IMGS_FOLDER, trainingData, responseData, label_to_ub_name);
					if (label_to_ub_name.size() > 1)
					{
						save_avg_hists(trainingData, responseData, label_to_ub_name, COLOR_CLASSIFIER);
						write_yaml<std::string>(U_BODY_FOLDER, "dc_data_status.yml", "status", "UPDATED");
						write_vector(U_BODY_NAMES, label_to_ub_name);
						classifier_exists = true;
					}
				}
				else	classifier_exists = true;
			}

			std::vector<std::string> ub_predictions;
			std::vector<double> ub_depths;
			std::vector<double> ub_midpts;
			std::vector<double> unknown_ubdepths;
			std::vector<double> unknown_ubmidpts;


			for (unsigned int i = 0; i < ub_rects.size(); i++)
			{
				cv::Mat hue_hist(256, 1, CV_32F, 0.0);
				cv::Rect dc_rect;
				dc_rect = extract_dress_area(ub_rects[i]);
				cv::Mat dc_img;
				dc_img = img_bgr(dc_rect);
				calc_hist(dc_img, hue_hist);
				std::string ub_prediction = "unknown";

				if (classifier_exists)
					ub_prediction = classify_hue(hue_hist, COLOR_CLASSIFIER);

				if (ub_rects.size() == 1
					&& predicted_names.size() == 1
					&& (ub_prediction == "unknown" || ub_prediction == predicted_names[0]))
				{

					int dir_count = count_files(DRESS_COLOR_IMGS_FOLDER + predicted_names[0]);
					if (dir_count < 30)
					{
						append_to_yaml(HUE_HISTS_FOLDER, predicted_names[0], hue_hist, dir_count + 1);
						save_image(dc_img, DRESS_COLOR_IMGS_FOLDER, predicted_names[0], dir_count + 1);
						write_yaml<std::string>(U_BODY_FOLDER, "dc_data_status.yml", "status", "CHANGED");
					}
				}

				cv::Mat ub_depth = depth_msg(dc_rect);
				double min_depth = 0;
				double x_real = 0;

				if (cv::countNonZero(ub_depth) > 0.25 * ub_depth.size().area())
				{
					cv::Point min_xy;
					cv::minMaxLoc(ub_depth, &min_depth, NULL, &min_xy, NULL, ub_depth != 0);
					min_depth /= 1000;
					x_real = -1 * (dc_rect.x + min_xy.x - (frame_width / 2))
						* (min_depth / focal_length);
				}

				if (ub_prediction == "unknown")
				{
					unknown_ubdepths.push_back(min_depth);
					unknown_ubmidpts.push_back(x_real);
				}
				else
				{
					if (!FIND_SUCCESS && ub_prediction == fdr_goal->name)
					{
						FIND_SUCCESS = true;
						target_depth = min_depth;
						target_midpt = x_real;
					}
					ub_predictions.push_back(ub_prediction);
					ub_midpts.push_back(x_real);
					ub_depths.push_back(min_depth);
				}
				if (fdr_goal->order == "RECOGNIZE_PCA_HAAR" ||
					fdr_goal->order == "RECOGNIZE_PCA_LBP")
				{
					fdr_feedback.order = fdr_goal->order;
					fdr_feedback.name = ub_prediction;
					fdr_feedback.result = "dress color";
					fdr_feedback.depth = min_depth;
					fdr_feedback.xmidpt = x_real;
					fdr_as.publishFeedback(fdr_feedback);
					if (ub_prediction != "unknown")
					{
						tf::Vector3 vec_tmp(min_depth + 1, x_real, 0.0);
						tf::Vector3 vec_sa = transform_sa(vec_tmp);
						cv::Point2d tpt(vec_sa.x(), vec_sa.y());
						std::string path_ext = ub_prediction + "/last_seen.yml";
						write_yaml_2<double, cv::Point2d>(FACE_IMGS_FOLDER, path_ext, "time", ros::WallTime::now().toSec(), "location", tpt);
					}
				}

			}

			if (fdr_goal->order == "FIND_AND_FOLLOW_HAAR" ||
				fdr_goal->order == "FIND_AND_FOLLOW_LBP")
			{
				if (!FIND_SUCCESS)
				{
					std::string name_ext = fdr_goal->name + "/last_seen.yml";
					boost::filesystem::path file_path(FACE_IMGS_FOLDER + name_ext);
					if (boost::filesystem::exists(file_path))
					{
						double time_seen;
						cv::Point2d loc_seen;
						read_yaml_2<double, cv::Point2d>(FACE_IMGS_FOLDER, name_ext, "time", time_seen, "location", loc_seen);
						fdr_feedback.order = fdr_goal->order;
						fdr_feedback.name = fdr_goal->name;
						fdr_feedback.result = "STORED";
						fdr_feedback.time_seen = time_seen;
						fdr_feedback.depth = loc_seen.x;
						fdr_feedback.xmidpt = loc_seen.y;
						fdr_as.publishFeedback(fdr_feedback);
					}
					else
					{
						fdr_feedback.order = fdr_goal->order;
						fdr_feedback.name = fdr_goal->name;
						fdr_feedback.result = "FAIL";
						fdr_as.publishFeedback(fdr_feedback);
					}

				}
				else
				{
					tf::StampedTransform transform;
					try
					{
						listener.waitForTransform("/map", "/camera_rgb_frame", img_time, ros::Duration(1.0));
						listener.lookupTransform("/map", "/camera_rgb_frame", img_time, transform);
					}
					catch (tf::TransformException ex)
					{
						ROS_ERROR("%s", ex.what());
					}
					tf::Vector3 vec_tmp(target_depth - 1, target_midpt, 0.0);
					tf::Vector3 vec = transform(vec_tmp);
					fdr_feedback.order = fdr_goal->order;
					fdr_feedback.name = fdr_goal->name;
					fdr_feedback.result = "SUCCESS";
					fdr_feedback.xmidpt = vec.y();
					fdr_feedback.depth = vec.x();
					fdr_as.publishFeedback(fdr_feedback);

				}
			}
		}

		CBrate.sleep();
	}

	else if (fdr_goal->order == "FIND_OBJECT")
	{

		ros::Rate CBrate(2.0);
		ros::Time img_time = rgb_img_ptr->header.stamp;
		cv::Mat img = cv_bridge::toCvCopy(rgb_img_ptr, "mono8")->image;
		cv::Mat depth_img = cv_bridge::toCvCopy(depth_img_ptr, "32FC1")->image;

		cv::SurfFeatureDetector detector;
		cv::SurfDescriptorExtractor extractor;
		cv::Mat q_descriptors;
		std::vector<cv::KeyPoint> q_keypoints;
		std::vector<cv::Point2f> q_corners;

		cv::Mat q_img = cv::imread(fdr_goal->name, CV_LOAD_IMAGE_GRAYSCALE);
		if (!q_img.data)
		{
			ROS_INFO("error reading image");
			return;
		}
		detector.detect(q_img, q_keypoints);
		if (q_keypoints.size() < 2) return;
		extractor.compute(q_img, q_keypoints, q_descriptors);


		q_corners.push_back(cvPoint(0, 0));
		q_corners.push_back(cvPoint(q_img.cols, 0));
		q_corners.push_back(cvPoint(q_img.cols, q_img.rows));
		q_corners.push_back(cvPoint(0, q_img.rows));

		std::vector<cv::KeyPoint> keypoints;
		cv::Mat descriptors;
		detector.detect(img, keypoints);
		if (keypoints.size() < 2) return;
		extractor.compute(img, keypoints, descriptors);

		cv::FlannBasedMatcher matcher;
		float thresholdMatchingNN = 0.7;
		unsigned int thresholdGoodMatches = 8;

		std::vector< std::vector<cv::DMatch> > matches;

		matcher.knnMatch(q_descriptors, descriptors, matches, 2);

		std::vector<cv::DMatch> good_matches;

		std::vector<cv::Point2f> obj;
		std::vector<cv::Point2f> scene;
		std::vector<cv::Point2f> scene_corners(4);
		cv::Mat img_matches;


		for (int i = 0; i < std::min(descriptors.rows - 1, (int)matches.size()); i++)
		{
			if (matches[i][0].distance < thresholdMatchingNN*(matches[i][1].distance) &&
				(int)matches[i].size() <= 2 &&
				(int)matches[i].size() > 0)
			{
				good_matches.push_back(matches[i][0]);
			}
		}

		if (good_matches.size() < 3) return;

		cv::drawMatches(q_img, q_keypoints, img, keypoints, good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

		double min_depth = 0;
		double xmidpt = 0;
		bool OBJECT_FOUND = false;

		if (good_matches.size() >= thresholdGoodMatches)
		{
			for (unsigned int i = 0; i < good_matches.size(); i++)
			{
				obj.push_back(q_keypoints[good_matches[i].queryIdx].pt);
				scene.push_back(keypoints[good_matches[i].trainIdx].pt);
			}

			cv::Mat H = cv::findHomography(obj, scene, CV_RANSAC);

			cv::perspectiveTransform(q_corners, scene_corners, H);

			float ulx = std::max(scene_corners[0].x, scene_corners[3].x);
			float uly = std::max(scene_corners[0].y, scene_corners[1].y);
			float rwidth = std::min(scene_corners[1].x, scene_corners[2].x) - std::max(scene_corners[0].x, scene_corners[3].x);
			float rheight = std::min(scene_corners[2].y, scene_corners[3].y) - std::max(scene_corners[0].y, scene_corners[1].y);

			if (rwidth <= 0 || rheight <= 0 || ulx < 0 || uly < 0 ||
				ulx + rwidth >= depth_img.size().width || uly + rheight >= depth_img.size().height)

				return;

			cv::Rect ROI(ulx, uly, rwidth, rheight);

			cv::Mat obj_depth = depth_img(ROI);


			if (cv::countNonZero(obj_depth) > 0.25 * obj_depth.size().area())
			{
				cv::Point min_xy;
				cv::minMaxLoc(obj_depth, &min_depth, NULL, &min_xy, NULL, obj_depth != 0);
				min_depth /= 1000;
				xmidpt = -1 * (ROI.x + min_xy.x - (frame_width / 2))
					* (min_depth / focal_length);
				OBJECT_FOUND = true;
			}
		}
		if (OBJECT_FOUND)
		{
			cv::putText(img_matches, "Object Found", cvPoint(10, 50), cv::FONT_HERSHEY_COMPLEX_SMALL, 2, cvScalar(0, 0, 250), 1, CV_AA);
			cv::line(img_matches, scene_corners[0] + cv::Point2f(q_img.cols, 0), scene_corners[1] + cv::Point2f(q_img.cols, 0), cv::Scalar(0, 255, 0), 4);
			cv::line(img_matches, scene_corners[1] + cv::Point2f(q_img.cols, 0), scene_corners[2] + cv::Point2f(q_img.cols, 0), cv::Scalar(0, 255, 0), 4);
			cv::line(img_matches, scene_corners[2] + cv::Point2f(q_img.cols, 0), scene_corners[3] + cv::Point2f(q_img.cols, 0), cv::Scalar(0, 255, 0), 4);
			cv::line(img_matches, scene_corners[3] + cv::Point2f(q_img.cols, 0), scene_corners[0] + cv::Point2f(q_img.cols, 0), cv::Scalar(0, 255, 0), 4);
			tf::Vector3 vec_tmp(min_depth, xmidpt, 0.0);
			tf::Vector3 vec;
			if (!STANDALONE)
			{
				tf::StampedTransform transform;
				try
				{
					listener.waitForTransform("/map", "/camera_rgb_frame", img_time, ros::Duration(1.0));
					listener.lookupTransform("/map", "/camera_rgb_frame", img_time, transform);
				}
				catch (tf::TransformException ex)
				{
					ROS_ERROR("%s", ex.what());
				}

				vec = transform(vec_tmp);
			}

			else
			{
				vec = transform_sa(vec_tmp);
			}

			fdr_feedback.order = fdr_goal->order;
			fdr_feedback.name = fdr_goal->name;
			fdr_feedback.result = "SUCCESS";
			fdr_feedback.xmidpt = vec.y();
			fdr_feedback.depth = vec.x();
			fdr_as.publishFeedback(fdr_feedback);
		}
		else
		{
			fdr_feedback.order = fdr_goal->order;
			fdr_feedback.name = fdr_goal->name;
			fdr_feedback.result = "FAIL";
			fdr_feedback.xmidpt = 0;
			fdr_feedback.depth = 0;
			fdr_as.publishFeedback(fdr_feedback);

			cv::putText(img_matches, "", cvPoint(10, 50), cv::FONT_HERSHEY_COMPLEX_SMALL, 3, cvScalar(0, 0, 250), 1, CV_AA);
		}

		cv::imshow("Object detection", img_matches);
		cv::waitKey(1);
		CBrate.sleep();

	}


	else
	{
		ROS_INFO("%s", (fdr_goal->order).c_str());
		ROS_INFO("Unrecognized order. Use one of the following: 'DETECT_FACES_HAAR' or 'DETECT_FACES_LBP' or 'TRAIN_PCA' or 'RECOGNIZE_PCA_HAAR' or 'RECOGNIZE_PCA_LBP' or 'FIND_AND_FOLLOW_HAAR' or 'FIND_AND_FOLLOW_LBP' or 'FIND_OBJECT'.");
		preemptCB();
	}
}
