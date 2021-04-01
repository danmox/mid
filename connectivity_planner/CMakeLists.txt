cmake_minimum_required(VERSION 2.8.12)
project(connectivity_planner)

set(PKG_DEPENDS
  rospy
  routing_msgs
)
set(BUILD_DEPENDS
  ${PKG_DEPENDS}
)
set(RUN_DEPENDS
  ${PKG_DEPENDS}
)

find_package(catkin REQUIRED ${BUILD_DEPENDS})

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS ${RUN_DEPENDS}
)

install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)