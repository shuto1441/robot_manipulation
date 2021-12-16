cmake_minimum_required(VERSION 3.0.2)
project(robot_manipulation)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
)

add_message_files(
    FILES
    pack_current_position.msg
    pack_predicted_position.msg
    dobot_orbit.msg

 )

include_directories(
  ${catkin_INCLUDE_DIRS}
)

generate_messages(
  DEPENDENCIES
  std_msgs  
)
