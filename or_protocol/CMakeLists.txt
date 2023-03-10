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

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES or_protocol
  CATKIN_DEPENDS
    or_protocol_msgs
    rosbag
    roscpp
  DEPENDS Threads
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(LIBRARY_FILES
  src/bcast_socket.cpp
  src/network_state.cpp
  src/or_node.cpp
  src/utils.cpp
)

add_library(${PROJECT_NAME} ${LIBRARY_FILES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Threads::Threads)

add_executable(send_node src/send_node.cpp)
target_link_libraries(send_node ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(recv_node src/recv_node.cpp)
target_link_libraries(recv_node ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(ping_node src/ping_node.cpp)
target_link_libraries(ping_node ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(serialization_example src/serialization_example.cpp)
target_link_libraries(serialization_example ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(routing_example src/routing_example.cpp)
target_link_libraries(routing_example ${catkin_LIBRARIES} yaml-cpp ${PROJECT_NAME})

catkin_add_gtest(${PROJECT_NAME}-test test/serialization_test.cpp)
if (TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES} yaml-cpp ${PROJECT_NAME})
endif()
