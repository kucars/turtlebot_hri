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
-sound_play ROS package
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


//Used for map xy coordinates.
struct XYs
{
	double x;
	double y;
};

/*
The search path of the robot when it receives a "FIND_AND_FOLLOW_HAAR" or "FIND_AND_FOLLOW_LBP"
command is defined by vector of waypoints, through which the robot will keep moving until it finds the target
person.
One extension of this, is to have a search path defined for each room in your environment; and then
allow the user to oreder the robot to search a specific room. You will need to edit some of code though.

YOU SHOULD CHANGE THESE POINTS TO MATCH YOUR SETTING.

*/
const std::vector<XYs> WAYPOINTS = { { 2.59, -0.12 }, { 3.67, 2.48 }, { 1.62, 1.84 }, { 1.77, -0.47 } };


/*
Parameters for point cloud following (see turtlebot_follower package in ROS - that is what we use).
If you follow the setup correctly, these parameters should refer to the upper head-level camera.
Tune them if needed.
*/
const float min_x = -0.3;
const float max_x = 0.3;
const float min_y = -0.4;
const float max_y = 0.2;
const float max_z = 2.5;
const float goal_z = 1.0;



/*
The client class actually starts two clients: for the face recognition server (which also does object recognition), the other for move_base (navigation).
*/
class FDRClient
{
	ros::NodeHandle nh;

	FDR_CLIENT fdr_ac; //recognition client
	MOVE_CLIENT fdr_mc; //navigation client


	//goal msg (defined in the fdr.action file) to communicate with recognition server.
	face_dr::fdrGoal face_goal;

	//sends navigation goals
	move_base_msgs::MoveBaseGoal move_goal;

	/*
	When we are not sending navigation goals (i.e. during in-place rotation & point cloud following),
	we rely on velcoity commands, sent through vel_msg & vel_pub.
	*/
	geometry_msgs::Twist vel_msg;
	ros::Publisher vel_pub;

	/*
	Subscribes to fdrOrder (or whatever you call it using command-line args).
	This is how we currently communicate with the client: by publishing orders on this topic.
	Ideally we should be able to communicate with the robot (client) by voice commands,
	but so far we do not have that functionality.
	The msg published over this topic is defined in fdr_msg. It has 2 strings, 1 int (when any is not used, I replace it with "x"/0).

	The client accept the following orders:
	"DETECT_FACES_HAAR" "<name of person>" <number of samples to collect>
	"DETECT_FACES_LBP"  "<name of person>" <number of samples to collect>
	"TRAIN_PCA" "x" 0
	"RECOGNIZE_PCA_HAAR" "x" 0
	"RECOGNIZE_PCA_LBP" "x" 0
	"FIND_AND_FOLLOW_HAAR" "x" 0
	"FIND_AND_FOLLOW_LBP" "x" 0
	"FIND_OBJECT" "<path to sample image>" 0
	*/
	ros::Subscriber fdr_order_sub;

	//Used for following target person
	ros::Subscriber cloud_sub;

	//sound_play to have the robot say comments as it moves around
	//to listen to these comments, you should run sound_play_node on computer with speaker
	sound_play::SoundClient say_stuff;

	//these variables and flags are used to control the transition from one process/state to another.
	unsigned int move_goal_id;
	bool NEW_GOAL;
	bool FOUND;
	bool REACHED;
	bool LATCHED;
	ros::WallTime time_found;
	ros::WallTime time_reached;
	ros::WallTime time_cloud;
	std::string last_name;
	std::string last_order;
	double time_checked;
	bool STANDALONE;
	boost::mutex mtx_;

public:

	/*
	A client object is constructed with 3 arguments:
	-the corresponding face server name
	-the topic name on which it receives orders
	-a third string indicating wether the client is run on a turtlebot, or on a standalone (fixed) camera:
	---if the third string = "true", the client is run in standalone mode, otherwise in turtlebot.
	These arguments are passed as commnad-line args when you rosrun the node.
	*/
	FDRClient(std::string, std::string, std::string);

	//actionlib callbacks (activation, goal completion, feedback). (see actionlib docs for details)
	void face_active_cb();
	void face_done_cb(AC_GOAL_STATE, FDR_ACTION_RESULT_Ptr);
	void face_feedback_cb(FDR_ACTION_FEEDBACK_Ptr);

	//where you order the client by publsihing commands
	void fdr_order_cb(FDR_ORDER_Ptr);

	//point cloud callback to follow target person (runs only in turtlebot mode)
	void cloud_cb(const PointCloud::ConstPtr&);

	//move_base actionlib client goal completion callback
	void move_result_cb(AC_GOAL_STATE, MOVE_RESULT_Ptr);

};

#endif
