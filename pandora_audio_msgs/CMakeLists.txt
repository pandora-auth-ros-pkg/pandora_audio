cmake_minimum_required(VERSION 2.8.3)
project(pandora_audio_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  pandora_common_msgs
  )

add_message_files(
  FILES
  AudioData.msg
  SoundAlert.msg
  SoundAlertVector.msg
  Recognition.msg
  )

generate_messages(
  DEPENDENCIES
    std_msgs
    pandora_common_msgs
  )

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
    pandora_common_msgs
  )
