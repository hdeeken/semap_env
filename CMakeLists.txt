cmake_minimum_required(VERSION 2.8.3)
project(semap_env)

find_package(catkin REQUIRED COMPONENTS
  rospy
  interactive_markers
  semap_ros
  message_generation
)

add_service_files(DIRECTORY srv)

catkin_python_setup()

generate_messages(
DEPENDENCIES
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
  rospy
  interactive_markers
  semap_ros
  message_runtime
  DEPENDS
)

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

catkin_install_python(PROGRAMS src/test/test_imports.py
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})

catkin_install_python(PROGRAMS src/nodes/services.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
