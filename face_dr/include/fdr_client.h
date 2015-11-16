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
Last updated: 16.11.2015

The code below is inspired by, builds upon, and/or uses code from the following source(s):

    -face_recognition ROS package by Pouyan Ziafati, shared under a Creative Commons Attribution 3.0 license.
    -ROS turtlebot_follower package.
*/

#ifndef FDR_CLIENT__H
#define FDR_CLIENT__H

#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "sound_play/sound_play.h"
#include "face_dr/fdr_msg.h"
#include "face_dr/fdrAction.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_datatypes.h"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "fdr_utilities.h"

typedef actionlib::SimpleActionClient<face_dr::fdrAction> FDR_CLIENT;
typedef const actionlib::SimpleClientGoalState& AC_GOAL_STATE;
typedef const face_dr::fdrResultConstPtr& FDR_ACTION_RESULT_Ptr;
typedef const face_dr::fdrFeedbackConstPtr& FDR_ACTION_FEEDBACK_Ptr;
typedef const face_dr::fdr_msgConstPtr& FDR_ORDER_Ptr;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MOVE_CLIENT;
typedef const move_base_msgs::MoveBaseResultConstPtr& MOVE_RESULT_Ptr;

struct XYs
{
    double x;
    double y;    
};

const std::vector<XYs> WAYPOINTS = { {2.59, -0.12}, {3.67, 2.48}, {1.62, 1.84}, {1.77, -0.47} };
const float min_x = -0.3;
const float max_x = 0.3;
const float min_y = -0.4;
const float max_y = 0.2;
const float max_z = 1.5;
const float goal_z = 1.0;


class FDRClient
{
   ros::NodeHandle nh;

   FDR_CLIENT fdr_ac;
   MOVE_CLIENT fdr_mc;

   face_dr::fdrGoal face_goal;
   move_base_msgs::MoveBaseGoal move_goal;
   geometry_msgs::Twist vel_msg;

   ros::Publisher vel_pub;
   ros::Subscriber fdr_order_sub;
   ros::Subscriber cloud_sub;
   
   sound_play::SoundClient say_stuff;
   unsigned int move_goal_id;
   bool NEW_GOAL;
   bool FOUND;
   bool REACHED;
   bool LATCHED;
   ros::Time time_found;
   ros::Time time_reached;
  
   ros::Time time_cloud;
   
   std::string last_name;
   std::string last_order;

   double time_checked;
   bool STANDALONE;

   boost::mutex mtx_;
   
public:

   FDRClient(std::string, std::string, std::string);
   void face_active_cb();
   void face_done_cb(AC_GOAL_STATE, FDR_ACTION_RESULT_Ptr);
   void face_feedback_cb(FDR_ACTION_FEEDBACK_Ptr);
   void fdr_order_cb(FDR_ORDER_Ptr);
   void cloud_cb(const PointCloud::ConstPtr&);
   void move_result_cb(AC_GOAL_STATE, MOVE_RESULT_Ptr);
   
};

#endif
