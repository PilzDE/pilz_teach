cmake_minimum_required(VERSION 2.8.3)
project(pilz_teleoperation)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  roslint
  std_msgs
)

add_service_files(
  FILES
  SetTeleopSettings.srv
)

catkin_python_setup()
generate_messages()
catkin_package(CATKIN_DEPENDS message_runtime)

catkin_install_python(PROGRAMS 
    scripts/key_teleop.py
    scripts/pilz_teleop_driver.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  file(GLOB integrationtest_files "test/integrationtests/*.test")
  file(GLOB integrationtest_py_files "test/integrationtests/*.py")
  foreach(file ${integrationtest_files})
    add_rostest(${file})
  endforeach()

  foreach(file ${integrationtest_py_files})
    roslint_python(${file})
  endforeach()

  file(GLOB modultests_files "test/modultests/tst_*.py")
  foreach(file ${modultests_files})
    catkin_add_nosetests(${file})
    roslint_python(${file})
  endforeach()

  file(GLOB python_files "src/${PROJECT_NAME}/*.py")
  foreach(file ${python_files})
    roslint_python(${file})
  endforeach()

  roslint_add_test()

  ######################
  ## Coverage Testing ##
  ######################

  function(ADD_PYTHON_COVERAGE)
    set(options NONE)
    set(oneValueArgs NAME)
    set(multiValueArgs DEPENDENCIES)
    cmake_parse_arguments(Coverage "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    add_custom_target(${Coverage_NAME}_cleanup
      # Cleanup coverage data from previous run
      COMMAND python-coverage erase
      WORKING_DIRECTORY $ENV{HOME}/.ros
      COMMENT "Resetting python code coverage counters to zero."
    )

    add_custom_target(${Coverage_NAME}
      COMMAND python-coverage combine
      COMMAND python-coverage report --include "*${PROJECT_SOURCE_DIR}*" --omit ${COVERAGE_EXCLUDES}
      COMMAND python-coverage html --include "*${PROJECT_SOURCE_DIR}*" --omit ${COVERAGE_EXCLUDES}
      WORKING_DIRECTORY $ENV{HOME}/.ros
      DEPENDS _run_tests_${PROJECT_NAME}
      COMMENT "Processing code coverage counters and generating report. Please find the generated report in $ENV{HOME}/.ros/htmlcov/index.html"
    )

    add_dependencies(_run_tests_${PROJECT_NAME} ${Coverage_NAME}_cleanup)
  endfunction() # SETUP_TARGET_FOR_COVERAGE

  # to run: catkin_make -DENABLE_COVERAGE_TESTING=ON package_name_coverage
  # Import errors may arise when packages are installed in the non-standard path /usr/lib/python2.7/dist-packages
  # This problem can be avoided by including the path in the PYTHONPATH environment variable
  if(ENABLE_COVERAGE_TESTING)
    set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test/*") #comma-sparated list of ignored patterns
    add_python_coverage(
      NAME ${PROJECT_NAME}_coverage
    )
  endif(ENABLE_COVERAGE_TESTING)
endif()
