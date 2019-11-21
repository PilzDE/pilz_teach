cmake_minimum_required(VERSION 2.8.3)
project(pilz_teleoperation)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
)


add_service_files(
  FILES
  SetTeleopSettings.srv
)

catkin_python_setup()
generate_messages()
catkin_package(CATKIN_DEPENDS message_runtime)

catkin_install_python(PROGRAMS scripts/key_teleop.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

include_directories(
    ${catkin_INCLUDE_DIRS}
)