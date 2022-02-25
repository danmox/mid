cmake_minimum_required(VERSION 3.0.2)
project(or_protocol)

add_compile_options(-std=c++11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

find_package(catkin REQUIRED)
find_package(Threads REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES or_protocol
#  CATKIN_DEPENDS other_catkin_pkg
  DEPENDS Threads
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(or_test src/test_node.cpp src/or_node.cpp src/bcast_socket.cpp)
target_link_libraries(or_test ${catkin_LIBRARIES} Threads::Threads)
