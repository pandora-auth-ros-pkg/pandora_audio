cmake_minimum_required(VERSION 2.8.3)
project(pandora_kinect_audio_processing)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  pandora_audio_msgs
  pandora_common_msgs
  state_manager
  state_manager_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    pandora_audio_msgs
    pandora_common_msgs
    state_manager
    state_manager_msgs
)

catkin_python_setup()
