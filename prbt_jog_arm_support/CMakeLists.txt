cmake_minimum_required(VERSION 2.8.3)
project(prbt_jog_arm_support)

find_package(catkin REQUIRED COMPONENTS
)

catkin_package()
# catkin_python_setup()

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  file(GLOB integrationtest_files "test/integrationtests/*.test")
  foreach(file ${integrationtest_files})
    add_rostest(${file})
  endforeach()

endif()
