cmake_minimum_required(VERSION 2.8.3)
project(pandora_voice_recognition)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  std_msgs
  rospy
  pandora_common_msgs
)




catkin_package(DEPENDS
  roscpp
  roslint
  std_msgs
  pandora_common_msgs
)

catkin_python_setup()

install(DIRECTORY demo
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS nodes/recognizer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(alert_generator src/alert_generator.cpp)
target_link_libraries(alert_generator ${catkin_LIBRARIES})



