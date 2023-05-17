cmake_minimum_required(VERSION 3.0.2)
project(or_protocol)

add_compile_options(-std=c++17)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

find_package(catkin REQUIRED COMPONENTS
  or_protocol_msgs
  rosbag
  roscpp
)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(fmt REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES or_protocol
  CATKIN_DEPENDS
    or_protocol_msgs
    rosbag
    roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(LIBRARY_FILES
  src/bcast_socket.cpp
  src/network_state.cpp
  src/or_protocol.cpp
  src/utils.cpp
)

add_library(${PROJECT_NAME} ${LIBRARY_FILES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Threads::Threads fmt::fmt)

add_executable(or_node src/or_node.cpp)
target_link_libraries(or_node ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(test_node src/test_node.cpp)
target_link_libraries(test_node ${catkin_LIBRARIES} ${PROJECT_NAME} yaml-cpp)

add_executable(ping_node src/ping_node.cpp)
target_link_libraries(ping_node ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(serialization_example src/serialization_example.cpp)
target_link_libraries(serialization_example ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(routing_example src/routing_example.cpp)
target_link_libraries(routing_example ${catkin_LIBRARIES} ${PROJECT_NAME} yaml-cpp)

catkin_add_gtest(${PROJECT_NAME}-test test/test.cpp)
if (TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES} ${PROJECT_NAME} yaml-cpp)
endif()
