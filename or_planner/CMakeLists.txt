cmake_minimum_required(VERSION 3.0.2)
project(or_planner)

add_compile_options(-std=c++17)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

set(BUILD_DEPS
  actionlib
  geometry_msgs
  nav_msgs
  or_protocol_msgs
  roscpp
  scarab_msgs
  sensor_msgs
  tf2
  visualization_msgs
)

find_package(catkin REQUIRED COMPONENTS
  ${BUILD_DEPS}
)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(fmt REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${BUILD_DEPS}
#  DEPENDS
#    Eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(LIBRARY_FILES
  src/or_planner.cpp
)

add_library(${PROJECT_NAME} ${LIBRARY_FILES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Eigen3::Eigen fmt::fmt)

add_executable(or_planner_node src/or_planner_node.cpp)
target_link_libraries(or_planner_node ${catkin_LIBRARIES} ${PROJECT_NAME})
