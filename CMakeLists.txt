cmake_minimum_required(VERSION 3.0.2)
project(robot_manipulation)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
)

catkin_python_setup()
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

catkin_package()

catkin_install_python(
  PROGRAMS
  nodes/tk_controller.py
  nodes/dobot_orbit_decider.py
  nodes/dualshock_controller.py
  nodes/image2position.py
  nodes/position_predictor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
