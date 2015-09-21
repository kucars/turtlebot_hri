/*
THIS IS AN EDITED VERSION OF THE ORIGINAL face_rec_client OF THE face_recognition PACKAGE. IN PARTICULAR, THIS VERSION INCOPORATES EVERYTHING WITHIN A CLASS, AND ADDS A PUBLISHER TO BE USED TO DIRECT THE MOTION OF THE TURTLEBOT. EDITING WAS DONE BY ALAA.
*/

/* Copyright 2012 Pouyan Ziafati, University of Luxembourg and Utrecht University

* actionlib client example for the face_recognition simple actionlib server. The client subscribes to face_recognition::FRClientGoal messages. Each FRClientGoal message contains an order_id and an order_argument which specify a goal to be executed by the face_recognition server. After receiving a message, the client sends the corresponding goal to the server. By registering relevant call back functions, the client receives feedback and result information from the execution of goals in the server and prints such information on the terminal.

*Provided by modifying the ROS wiki tutorial: "actionlib_tutorials/Tutorials/Writing a Callback Based Simple Action Client (last edited 2010-09-13 17:32:34 by VijayPradeep)"

*License: Creative Commons Attribution 3.0. 
*/

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <face_recognition/FRClientGoal.h>
#include <face_recognition/FaceRecognitionAction.h>
#include <signal.h>
#include "face_recognition/DAF.h"
#include "sound_play/sound_play.h"

typedef actionlib::SimpleActionClient<face_recognition::FaceRecognitionAction> Client;
face_recognition::FaceRecognitionGoal goal;
std::string hello_ = "Hello ";

class NodeInit
{
public:
   NodeInit(): ac("face_recognition", true), say_stuff(nh, "robotsound")
      {
         while (!ac.waitForServer(ros::Duration(0.0))) ROS_INFO("Waiting for server");
         motionPub = nh.advertise<face_recognition::DAF>("whatNext", 1);
         sub = nh.subscribe("fr_order", 1, &NodeInit::frclientCallback, this);
         prev_face = "none";
      }

   void doneCB(const actionlib::SimpleClientGoalState& state, const face_recognition::FaceRecognitionResultConstPtr& result)
   {
      ROS_INFO("Goal [%i] Finished in state [%s]", result->order_id,state.toString().c_str());
      if(state.toString() != "SUCCEEDED") return;
      if( result->order_id==0)
         ROS_INFO("%s was recognized with confidence %f", result->names[0].c_str(),result->confidence[0]);          
      if( result->order_id==2)
         ROS_INFO("Pictures of %s were successfully added to the training images",result->names[0].c_str());
   }


   void activeCb()
   {
      ROS_INFO("Goal just went active");
   }


   void feedbackCb(const face_recognition::FaceRecognitionFeedbackConstPtr& feedback)
   {
      ROS_INFO("Received feedback from Goal [%d] ", feedback->order_id);

      if(feedback->order_id==1 )
      {
         if (strcmp(feedback->names[0].c_str(), "none")==0)
         {
            ROS_INFO("Did not detect any face");
            motion_msg.detected_a_face = "MOVE";
            motionPub.publish(motion_msg);
         }
         else if (strcmp(feedback->names[0].c_str(), "Stranger")==0)
         {
            ROS_INFO("An unknown face was detected");
            motion_msg.detected_a_face = "STOP";
            motionPub.publish(motion_msg);
         }
         else
         {
            ROS_INFO("%s was recognized with confidence %f", feedback->names[0].c_str(),feedback->confidence[0]);
            motion_msg.detected_a_face = "STOP";
            motionPub.publish(motion_msg);
            if (prev_face!=feedback->names[0].c_str())
	    {
		say_stuff.say(hello_+feedback->names[0].c_str());
		prev_face = feedback->names[0].c_str();
	    } 
         }
      }    
      else if( feedback->order_id==2)
        ROS_INFO("A picture of %s was successfully added to the training images",feedback->names[0].c_str());
   }
   void frclientCallback(const face_recognition::FRClientGoalConstPtr& msg)
   {
          
      ROS_INFO("request for sending goal [%i] is received", msg->order_id);
      goal.order_id = msg->order_id;
      goal.order_argument = msg->order_argument;
      ac.sendGoal(goal, boost::bind(&NodeInit::doneCB, this, _1, _2), boost::bind(&NodeInit::activeCb, this), boost::bind(&NodeInit::feedbackCb, this, _1));        
   }

protected:
   Client ac;
   ros::NodeHandle nh;
   ros::Publisher motionPub;
   face_recognition::DAF motion_msg;
   ros::Subscriber sub;
   sound_play::SoundClient say_stuff;
   std::string prev_face;
};




int main(int argc, char** argv)
{
   ros::init(argc, argv, "face_recognition_client");
   NodeInit client_node;
   ros::spin();
   
   return 0;
}
