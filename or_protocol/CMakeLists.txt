cmake_minimum_required(VERSION 3.0.2)
project(or_protocol)

add_compile_options(-std=c++17)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  or_protocol_msgs
)
find_package(Threads REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES or_protocol
  CATKIN_DEPENDS
    or_protocol_msgs
  DEPENDS Threads
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(LIBRARY_FILES
  src/or_node.cpp
  src/bcast_socket.cpp
)

add_library(${PROJECT_NAME} ${LIBRARY_FILES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Threads::Threads)

add_executable(send_node src/send_node.cpp)
target_link_libraries(send_node ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(recv_node src/recv_node.cpp)
target_link_libraries(recv_node ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(ping_node src/ping_node.cpp)
target_link_libraries(ping_node ${catkin_LIBRARIES} ${PROJECT_NAME})

if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(serialization_test test/serialization_test.cpp)
  target_link_libraries(serialization_test ${catkin_LIBRARIES})
endif()
