cmake_minimum_required(VERSION 2.8.3)
project(face_dr)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs actionlib actionlib_msgs image_transport message_generation cv_bridge sound_play sensor_msgs message_filters tf move_base_msgs geometry_msgs nav_msgs pcl_ros)

find_package(Boost REQUIRED)
find_package(PCL)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

 add_message_files(FILES fdr_msg.msg)

 add_action_files(DIRECTORY action FILES fdr.action)

 generate_messages(DEPENDENCIES std_msgs actionlib_msgs)

catkin_package(
  INCLUDE_DIRS ${actionlib_msgs_PACKAGE_PATH}
  CATKIN_DEPENDS message_runtime roscpp std_msgs actionlib actionlib_msgs roslib cv_bridge image_transport sound_play sensor_msgs rospy message_filters tf move_base_msgs geometry_msgs nav_msgs pcl_ros
  DEPENDS Boost PCL
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${actionlib_msgs_PACKAGE_PATH}
  ${OpenCV_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(face_dr_lib src/fdr_utilities.cpp src/fdr_server.cpp src/fdr_client.cpp)
target_link_libraries(face_dr_lib ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} opencv_nonfree)
add_dependencies(face_dr_lib face_dr_generate_messages_cpp)

add_executable(fdr_client src/fdr_client_node.cpp)
add_dependencies(fdr_client face_dr_generate_messages_cpp)
add_dependencies(fdr_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(fdr_server src/fdr_server_node.cpp)
add_dependencies(fdr_server face_dr_generate_messages_cpp)
add_dependencies(fdr_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(fdr_client ${catkin_LIBRARIES} ${PCL_LIBRARIES} face_dr_lib)
target_link_libraries(fdr_server ${catkin_LIBRARIES} face_dr_lib)

install(
    TARGETS face_dr_lib 
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
