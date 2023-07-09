cmake_minimum_required(VERSION 3.0.2)
project(laserscan_merger)

add_compile_options(-std=c++17)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

set(BUILD_DEPS
  message_filters
  roscpp
  sensor_msgs
)

find_package(catkin REQUIRED COMPONENTS
  ${BUILD_DEPS}
)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(fmt REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    ${BUILD_DEPS}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(laserscan_merger src/laserscan_merger.cpp)
target_link_libraries(laserscan_merger ${catkin_LIBRARIES})
