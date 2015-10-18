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


Face detection and recognition ROS actionlib client.
Subscribes to:
   -Topic: "fdrOrder".
      -Callback: sub_cb(): sends action goal.
Publishes:
   -Names of recognized faces (through a sound_play::SoundClient class)

Author: Alaa El Khatib
Last updated: 08.10.2015

The code below is inspired by, builds upon, and/or uses code from the following source(s):

   -face_recognition ROS package by Pouyan Ziafati, shared under a Creative Commons Attribution 3.0 license.

*/

#ifndef FDR_CLIENT__H
#define FDR_CLIENT__H

#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "sound_play/sound_play.h"
#include "face_dr/fdr_msg.h"
#include "face_dr/motion_msg.h"
#include "face_dr/fdrAction.h"

typedef actionlib::SimpleActionClient<face_dr::fdrAction> FDR_CLIENT;
typedef const actionlib::SimpleClientGoalState& AC_GOAL_STATE;
typedef const face_dr::fdrResultConstPtr& FDR_ACTION_RESULT_Ptr;
typedef const face_dr::fdrFeedbackConstPtr& FDR_ACTION_FEEDBACK_Ptr;
typedef const face_dr::fdr_msgConstPtr& FDR_ORDER_Ptr;



class FDRClient
{
   ros::NodeHandle nh;
   FDR_CLIENT fdr_ac;
   ros::Subscriber fdr_order_sub;
   sound_play::SoundClient say_stuff;
   face_dr::fdrGoal goal;
   ros::Publisher fdr_motion_pub;
   face_dr::motion_msg move_msg; 
   std::string prev_face;

public:

   FDRClient();
   void active_cb();
   void done_cb(AC_GOAL_STATE, FDR_ACTION_RESULT_Ptr);
   void feedback_cb(FDR_ACTION_FEEDBACK_Ptr);
   void sub_cb(FDR_ORDER_Ptr);
   
};

#endif
