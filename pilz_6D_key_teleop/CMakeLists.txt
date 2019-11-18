cmake_minimum_required(VERSION 2.8.3)
project(pilz_6D_key_teleop)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package()

catkin_python_setup()

catkin_install_python(PROGRAMS scripts/key_teleop.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

include_directories(
    ${catkin_INCLUDE_DIRS}
)