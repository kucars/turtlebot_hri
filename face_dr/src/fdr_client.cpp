/*

Copyright 2015.

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
Last updated: 24.11.2015

The code below is inspired by, builds upon, and/or uses code from the following source(s):

	-face_recognition ROS package by Pouyan Ziafati, shared under a Creative Commons Attribution 3.0 license.
	-ROS turtlebot_follower package.
	-sound_play ROS package


*/


#include "fdr_client.h"


FDRClient::FDRClient(std::string s_name, std::string sub_name, std::string standalone) : fdr_ac(s_name, true), fdr_mc("move_base", true), say_stuff(nh, "robotsound")
{
	if (standalone == "true")
		STANDALONE = true;
	else
		STANDALONE = false;

	while (!fdr_ac.waitForServer(ros::Duration(0.0))) ROS_INFO("waiting for face server");
	if (!STANDALONE)
	{
		while (!fdr_mc.waitForServer(ros::Duration(0.0))) ROS_INFO("Waiting for move_base server");
	}
	fdr_order_sub = nh.subscribe(sub_name, 1, &FDRClient::fdr_order_cb, this);
	vel_pub = nh.advertise<geometry_msgs::Twist>("/raw_cmd_vel", 1, true);
	move_goal_id = 0;
	NEW_GOAL = true;
	time_cloud = ros::WallTime::now();
	time_checked = ros::WallTime::now().toSec();
}

void FDRClient::face_active_cb()
{
	ROS_INFO("FDR action goal went active");
}


void FDRClient::face_done_cb(AC_GOAL_STATE state, FDR_ACTION_RESULT_Ptr result)
{
	ROS_INFO("FDR action goal DONE in state %s ", state.toString().c_str());
	if (state.toString() == "SUCCEEDED")
	{
		if (result->order == "DETECT_FACES_HAAR" ||
			result->order == "DETECT_FACES_LBP")
			ROS_INFO("%d training images for %s saved", result->num_of_samples, (result->name).c_str());

		else if (result->order == "TRAIN_PCA")
			ROS_INFO("Training DONE");
	}
}


void FDRClient::face_feedback_cb(FDR_ACTION_FEEDBACK_Ptr feedback)
{
	
	if (feedback->order == "DETECT_FACES_HAAR" || feedback->order == "DETECT_FACES_LBP")

		ROS_INFO("%d image(s) added so far", (int)feedback->xmidpt);

	else if (feedback->order == "RECOGNIZE_PCA_HAAR" || feedback->order == "RECOGNIZE_PCA_LBP")
	{
		if (feedback->name != "none" && feedback->name != "unknown")
			ROS_INFO("%s was recognized", feedback->name.c_str());
	}

	else if (feedback->order == "FIND_AND_FOLLOW_HAAR" || feedback->order == "FIND_AND_FOLLOW_LBP")
	{
		boost::lock_guard<boost::mutex> guard(mtx_);

		if (!FOUND && (feedback->result == "FAIL" || (feedback->result == "STORED" && feedback->time_seen < time_checked)))
		{
			if (NEW_GOAL)
			{
				NEW_GOAL = false;
				move_goal.target_pose.header.frame_id = "map";
				move_goal.target_pose.header.stamp = ros::Time::now();
				move_goal.target_pose.pose.position.x = WAYPOINTS[move_goal_id].x;
				move_goal.target_pose.pose.position.y = WAYPOINTS[move_goal_id].y;
				move_goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI);
				ROS_INFO("Sending move goal to next waypoint");
				fdr_mc.sendGoal(move_goal, boost::bind(&FDRClient::move_result_cb, this, _1, _2));
			}
		}
		
		else if (!FOUND && (feedback->result == "SUCCESS" || (feedback->result == "STORED" && feedback->time_seen > time_checked)))
		{
			FOUND = true;
			REACHED = false;
			LATCHED = false;
			
			if (!NEW_GOAL)
				fdr_mc.cancelGoalsAtAndBeforeTime(ros::Time::now());
				 	
			ROS_INFO("%s is found at map coordinates (%f, %f)", last_name.c_str(), feedback->depth, feedback->xmidpt);
			move_goal.target_pose.pose.position.x = feedback->depth;
			move_goal.target_pose.pose.position.y = feedback->xmidpt;
			move_goal.target_pose.header.frame_id = "map";
			move_goal.target_pose.header.stamp = ros::Time::now();
			move_goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI);
			ROS_INFO("Sending move goal to target location");
			fdr_mc.sendGoal(move_goal, boost::bind(&FDRClient::move_result_cb, this, _1, _2));
			time_found = ros::WallTime::now();
			if (feedback->result == "STORED")
			{
				std::string speech_msg = "I found you " + last_name + ". I'll be there in a second.";
            	say_stuff.say(speech_msg);	
			}
		}
		
		else if (FOUND && !REACHED)
		{
			if (ros::WallTime::now() - time_found > ros::WallDuration(40))
			{
				ROS_INFO("Could not reach target location within 40 seconds");
				ROS_INFO("Resuming search path");
				FOUND = false;
				NEW_GOAL = true;
				time_checked = ros::WallTime::now().toSec();

			}
		}
		
		else if (REACHED && (feedback->result == "FAIL" || (feedback->result == "STORED" && feedback->time_seen < time_checked)) && !LATCHED) //checking for LATCHED should not be needed, but just in case a new feedback is available before server preempt
		{

			if (ros::WallTime::now() - time_reached > ros::WallDuration(40))
			{
				ROS_INFO("%s does not appear to be at expected location", last_name.c_str());
				ROS_INFO("Resuming search path");
				FOUND = false;
				REACHED = false;
				NEW_GOAL = true;
			}
			else
			{
				vel_msg.linear.x = 0;
				vel_msg.angular.z = 0.5;
				vel_pub.publish(vel_msg);
			}
		}
		
		else if (REACHED && feedback->result == "SUCCESS" && !LATCHED)
		{

			vel_msg.linear.x = 0;
			vel_msg.angular.y = 0;
			vel_pub.publish(vel_msg);
			LATCHED = true;
			fdr_ac.cancelGoalsAtAndBeforeTime(ros::Time::now());
			cloud_sub = nh.subscribe<PointCloud>("/facecamera/depth_registered/points", 1, &FDRClient::cloud_cb, this);
			std::string speech_msg = "Hello " + last_name + ". How can I help you?";
            say_stuff.say(speech_msg);
			
		}
		
	}


	else if (feedback->order == "FIND_OBJECT")
	{
		if (feedback->result == "SUCCESS")
			ROS_INFO("object was found at map coordinates (%f, %f)", feedback->depth, feedback->xmidpt);
		else
			; //do nothing
	}
}

void FDRClient::move_result_cb(AC_GOAL_STATE state, MOVE_RESULT_Ptr result)
{
	
	boost::lock_guard<boost::mutex> guard(mtx_);
	
	if (!FOUND) //i.e. we are in waypoint search
	{
		NEW_GOAL = true;
		move_goal_id = move_goal_id == WAYPOINTS.size() - 1 ? 0 : ++move_goal_id;
	}

	else if (state.toString() == "SUCCEEDED")
	{
		ROS_INFO("Target location reached");
		REACHED = true;
		time_reached = ros::WallTime::now();
		time_checked = time_reached.toSec();
	}
	
}

void FDRClient::cloud_cb(const PointCloud::ConstPtr& cloud)
{
	if (LATCHED)
	{
		
		boost::lock_guard<boost::mutex> guard(mtx_);
		
		float x = 0.0;
		float y = 0.0;
		float z = 1e6;
		unsigned int n = 0;
		BOOST_FOREACH(const pcl::PointXYZ& pt, cloud->points)
		{
			if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z))
			{
				if (-pt.y > min_y && -pt.y < max_y && pt.x < max_x && pt.x > min_x && pt.z < max_z)
				{
					x += pt.x;
					y += pt.y;
					z = std::min(z, pt.z);
					n++;
				}
			}
		}
		if (n > 100)
		{
			x /= n;
			y /= n;
			if (fabs(z - goal_z) > 0.1)
			{
				vel_msg.linear.x = (z - goal_z) * 1;
			}

			else
			{
				vel_msg.linear.x = 0;	
			}

			vel_msg.angular.z = -1 * x * 2.3;
			vel_pub.publish(vel_msg);
			time_cloud = ros::WallTime::now();
		}
		else
		{
			ROS_INFO("Not enough points within point cloud range");
			ROS_INFO("Waiting for more...");
			if (ros::WallTime::now() - time_cloud > ros::WallDuration(10.0))
			{
				
				std::string speech_msg = "Oh, it seems I have lost you, " + last_name + ". I'll try to find you again.";
            	say_stuff.say(speech_msg);
				FOUND = true;
				REACHED = true;
				LATCHED = false;
				time_reached = ros::WallTime::now();
				face_goal.order = last_order;
				face_goal.name = last_name;
				face_goal.num_of_samples = 0;
				fdr_ac.sendGoal(face_goal, boost::bind(&FDRClient::face_done_cb, this, _1, _2), boost::bind(&FDRClient::face_active_cb, this), boost::bind(&FDRClient::face_feedback_cb, this, _1));
				cloud_sub.shutdown();
			}
		}
		
	
	}
}

void FDRClient::fdr_order_cb(FDR_ORDER_Ptr msg)
{
	ROS_INFO("FDR order %s received", (msg->order).c_str());
	if (msg->order == "FIND_AND_FOLLOW_HAAR" ||
		msg->order == "FIND_AND_FOLLOW_LBP")
	{ 
		
		boost::lock_guard<boost::mutex> guard(mtx_);
		
		FOUND = false;
		REACHED = false;
		LATCHED = false;
		last_order = msg->order;
		last_name = msg->name;
		std::string speech_msg = "I am looking for " + msg->name;
        say_stuff.say(speech_msg);
		
	
	}
	face_goal.order = msg->order;
	face_goal.name = msg->name;
	face_goal.num_of_samples = msg->num_of_samples;
	fdr_ac.sendGoal(face_goal, boost::bind(&FDRClient::face_done_cb, this, _1, _2), boost::bind(&FDRClient::face_active_cb, this), boost::bind(&FDRClient::face_feedback_cb, this, _1));
}
