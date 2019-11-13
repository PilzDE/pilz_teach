cmake_minimum_required(VERSION 2.8.3)
project(pilz_jog_arm_support)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
)

catkin_package()
# catkin_python_setup()