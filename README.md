# Turtlebot HRI
Human detection, recognition and following, using multiple robots/sensors.


This code was run on Ubuntu 14.04/ROS Indigo 

Prerequisites:
This ROS package provides face/object recognition/following assuming the following:
You have already mapped the environment in which the Turtlebot will navigate.
The Turtlebot has been modified to add a second Kinect camera at head-level.
There is a standalone (fixed) Kinect camera somewhere in the environment.
You have already found the transform from the standalone camera frame to the map frame.
The code uses compressed images over using image_transport; make sure you have that installed.
Depending on how you run it (see below) you could do without one of the two component (the Turtlebot and the standalone camera).

Installation:
Assuming you have already installed ROS, OpenCV, etc., clone this repository into your catkin workspace. Before you catkin_make, you should probably edit some things in the files (see below).
The object recognition used uses SURF, which is part of the nonfree OpenCV libraries. You need to separately install it.

    sudo add-apt-repository --yes ppa:xqms/opencv-nonfree
    sudo apt-get update
    sudo apt-get install libopencv-nonfree-dev

Details:
This package was developed to provide the following functionalities:
Face detection and recognition.
Upper body detection, from which dress color is extracted.
Dress color and face info to be used for human identification.
Human following.
Object recognition.
The package was developed as part of a project for helping elderly people in their daily living, and these are some of the things we would want a robot caring for elderly people to do.

Run it:
First a few details need to be edited in the code.
Open the fdr_server.h file, and choose the ROS_DATA file path as you want it, and make sure you create the file tree shown yourself (i.e., go to these destinations and create the folders with the correct names).
Find OpenCV’s cascade classifier files (the ones used in named in the file tree) and copy them to the designated destination.
When you run the face recognition using your dataset, the PCA FaceRecognizer threshold of OpenCV’s implementation may not be suitable for you, and you will probably need to tune it to get a satisfactory recognition accuracy. That threshold is in fdr_server.h (called PCA_THRESH).
If you are using a standalone camera, you should find the transform from the camera frame to the map frame, and edit the value in the fdr_server.cpp file (the place where the edit goes is clearly commented, around line 100).
In fdr_client.h, change the WAYPOINTS to use your own points on your map.
We assume the following configuration (although you could have a different one):
The standalone camera is connected to the workstation computer.
The Turtlebot (and its 2 cameras) are connected to the Turtlebot laptop.
Make sure the network is configured correctly (select the ROS_MASTER ip address and so on).

On the Turtlebot laptop (SSH from workstation):

    roslaunch turtebot_bringup minimal.launch


(USE THE MAP FILE YOU HAVE.)

    roslaunch turtlebot_navigation amcl_demo.launch map_file:=/map.yaml


(MAKE SURE YOU HAVE depth_registration=true IN THE LAUNCH FILE FOR THIS CAMERA, OR USE COMMAND-LINE ARGS TO SET IT. YOU CAN USE WHATEVER PACKAGE YOU WANT TO RUN THE CAMERA, AS LONG AS DEPTH REGISTRATION IS ON! YOU WILL NEED TO CHOOSE DIFFERENT NAMES FOR THE CAMERA. ALSO, YOU WILL have TO SELECT THE RIGHT CAMERA, SINCE YOU HAVE 2 OF THEM CONNECTED TO THE LAPTOP. YOU CAN EITHER CONNECT ONE AND LAUNCH THE amcl_demo, AND THEN CONNECT THE OTHER; OR YOU CAN USE THE device_id ARG TO SELECT THE CAMERA YOU WANT. YOU WILL PROBABLY HAVE TO RESORT TO THE LATTER WAY EITHER WAY.)

    roslaunch freenect_launch freenect.launch camera:=facecamera


On the workstation:

    roslaunch turtlebot_rviz_launchers view_navigation.launch


(AGAIN MAKE SURE THE DEPTH REGISTRATION IS ON AND THAT YOU SELECT THE RIGHT CAMERA.)

    roslaunch freenect_launch freenect.launch camera:=sacamera


(RUN THE SERVERS FOR sacamera AND facecamera. THE SERVERS COULD RUN ANYWHERE, ON THE WORKSTATION OR IN THE CLOUD.)

    rosrun face_dr fdr_server "facecamera" "false" __name:=turtleServer
    rosrun face_dr fdr_server "sacamera" "true" __name:=saServer


On the Turtlebot laptop (SSH from workstation):

(CHOOSE THE CORRESPONDING SERVER NAME AND TOPIC NAME. “false”/“true” HERE REFER TO WHETHER THE CAMERA IS A STANDALONE CAMERA. THE VELOCITY COMMANDS TOPIC IS MAPPED TO THE NAVIGATION SMOOTHER, WHICH SHOULD BE THERE IF YOU RAN THE acml_demo. THIS CLIENT SHOULD BE RUN ON THE LAPTOP TO WHICH THE CAMERA IS CONNECTED, BECAUSE IT USES THE POINT CLOUD TO FOLLOW PEOPLE, AND IF IT IS NOT RUN LOCALLY, IT WILL BE TOO SLOW TO WORK.)

    rosrun face_dr fdr_client "turtleServer" "/fdrOrder1" "false" /raw_cmd_vel:=/navigation_velocity_smoother/raw_cmd_vel __name:=turtleClient


(THIS RUNS THE NODE THAT PLAYS THE STUFF THE ROBOT SAYS. THE SENETNCES ARE SYNTHESIED IN THE client NODE. MAKE SURE YOU HAVE THE sound_play PACKAGE.)

    rosrun sound_play soundplay_node.py


On the workstation (or the computer to which the standalone camera is connected):

    rosrun face_dr fdr_client "saServer" "/fdrOrder2" "true" __name:=saClient


To use the face recognition and human follower, you should first train the classifiers on some dataset of faces. To do that, publish the “DETECT_FACES_HAAR” or “DETECT_FACES_LBP” (depending on which features you want to use – see OpenCV’s documentation for details) on the fdrOrder1 or fdrOrder2 topic (it doesn’t matter if you do this using the standalone camera or the Turtlebot camera, so long as you run both servers on the same of computer and do not mess with file system hierarchy, and you stand in front of the correct camera!), with the name of the person you want to collect face samples for, and the number of samples you want to collect.

    rostopic pub -1 /fdrOrder1 face_dr/fdr_msg "DETECT_FACES_HAAR" "<name>" 50

You should now have a new folder in your file hierarchy with the collected face samples.
Repeat this for as many persons as you want.
Next, we should train the classifiers using the accumulated dataset. In our code we use Eigenfaces (PCA) to train and classify.

    rostopic pub -1 /fdrOrder2 face_dr/fdr_msg "TRAIN_PCA" "x" 0

Now the face classifier is ready to be used (for both cameras).

To start detection and recognition (and following mode):
On the topic of the standalone camera:

    rostopic pub -1 /fdrOrder2 face_dr/fdr_msg "RECOGNIZE_PCA_HAAR" "x" 0

On the topic of the Turtlebot camera:

    rostopic pub -1 /fdrOrder1 face_dr/fdr_msg "FIND_AND_FOLLOW_HAAR" "x" 0

(DO NOT INTERCHANGE THE TWO ORDERS—THE STANDALONE CAMERA CANNOT FOLLOW, AND THE RECOGNIZE_PCA_HAAR/LBP EXPECTS A MANUALLY ENTERED FRAME TRANSFORM.)

If you run everything as here, the robot should start moving along the path you selected using WAYPOINTS. The robot will be looking for you (the name you entered). As it (or the other camera) recognizes other people, it should be collecting dress color samples (found in the corresponding folder). Whichever camera detects you first, the robot will move towards your coordinates, and if it finds you there, it should start following you.

There is a minimum number of people in the dress color database that have to be reached before dress color is used to recognize people. The logic is this: say 4 people are in a house. The robot/camera saw 2 people one wearing green, one wearing red; it should not identify anyone wearing green as the person it saw earlier unless it already knows that no-one else is wearing green; hence, it has to wait until it sees everyone. To change that minimum number, look for 

    if (label_to_ub_name.size() > 1)

in fdr_server.cpp (around line 348) and change the limit. Currently the minimum number is 2. Remember to rebuild the workspace whenever you the source code!



Object recognition:

To recognize objects (using SURF), you need a sample image of the target object:

    rostopic pub -1 /fdrOrder1 face_dr/fdr_msg "FIND_OBJECT" "/path/to/img.jpg" 0

Note: Object recognition does not work well from afar. Also, if you alternate between orders, you may need to rerun the server/client every now and then. You can definitely fix that in code if you have the time!
