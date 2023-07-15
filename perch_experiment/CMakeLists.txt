cmake_minimum_required(VERSION 2.8.3)
project(perch_experiment)

set(PKG_DEPENDS
  exploration_msgs
  geometry_msgs
  or_protocol_msgs
  roscpp
  visualization_msgs
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

catkin_package(
  CATKIN_DEPENDS ${RUN_DEPENDS}
  DEPENDS
)

add_executable(exploration_vis src/exploration_visualizer.cpp)
target_link_libraries(exploration_vis ${catkin_LIBRARIES})

catkin_package()
