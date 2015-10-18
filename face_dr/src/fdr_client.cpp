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


#include "fdr_client.h"


FDRClient::FDRClient() : fdr_ac("fdr_server", true), say_stuff(nh, "robotsound")
{
   while (!fdr_ac.waitForServer(ros::Duration(0.0))) ROS_INFO("waiting for fdr_server");
   fdr_order_sub = nh.subscribe("/fdrOrder", 1, &FDRClient::sub_cb, this);
   fdr_motion_pub = nh.advertise<face_dr::motion_msg>("motionOrder", 1);
   prev_face = "none";
}

void FDRClient::active_cb()
{
   ROS_INFO("FDR action goal went active");
}


void FDRClient::done_cb(AC_GOAL_STATE state, FDR_ACTION_RESULT_Ptr result)
{
   ROS_INFO("FDR action goal DONE in state %s ", state.toString().c_str());
   if (state.toString()=="SUCCEEDED")
   {
      if (result->order == "DETECT_FACES_HAAR" ||
          result->order == "DETECT_FACES_LBP")
         ROS_INFO("%d training images for %s saved", result->num_of_samples, (result->name).c_str());

      else if (result->order == "TRAIN_PCA")
         ROS_INFO("Training DONE");
   }
}


void FDRClient::feedback_cb(FDR_ACTION_FEEDBACK_Ptr feedback)
{
   if (feedback->order == "DETECT_FACES_HAAR" ||
       feedback->order == "DETECT_FACES_LBP")
            ROS_INFO("%d image(s) added so far", (int)feedback->confidence);

   else if (feedback->order == "RECOGNIZE_PCA_HAAR" ||
            feedback->order == "RECOGNIZE_PCA_LBP")
   {
      if (feedback->name == "none")
      {
         //ROS_INFO("did not detect any face... publishing MOVE");
         move_msg.order = "MOVE";
         fdr_motion_pub.publish(move_msg);
      }
      else
      {
         if (feedback->name == "a stranger")
         {
            ROS_INFO("an unknown face was detected");
            move_msg.order = "STOP";
            fdr_motion_pub.publish(move_msg);
         }
         else if (feedback->name == "multiple UBs")
         {
            ROS_INFO("%d unidentified person(s) detected using upper body information", (int)feedback->confidence);
            move_msg.order = "STOP";
            fdr_motion_pub.publish(move_msg);
         }
         else
         {
            if (feedback->confidence < 0)
               ROS_INFO("%s was recognized using the color of his/her shirt, based on samples accumulated between %s and %s", (feedback->name).c_str(), (feedback->time_first).c_str(), (feedback->time_last).c_str());
            else
               ROS_INFO("%s was recognized with confidence %f", (feedback->name).c_str(), feedback->confidence);

            move_msg.order = "STOP";
            fdr_motion_pub.publish(move_msg);
            if (feedback->name != prev_face)
            {
               prev_face = feedback->name;
               std::string say_this = "Hello "+(feedback->name);
               say_stuff.say(say_this);
            }
         }
      }  
   }
}





void FDRClient::sub_cb(FDR_ORDER_Ptr msg)
{
   ROS_INFO("FDR order %s received", (msg->order).c_str());
   goal.order = msg->order;
   goal.name = msg->name;
   goal.num_of_samples = msg->num_of_samples;
   fdr_ac.sendGoal(goal, boost::bind(&FDRClient::done_cb, this, _1, _2), boost::bind(&FDRClient::active_cb, this), boost::bind(&FDRClient::feedback_cb, this, _1));
}
