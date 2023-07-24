cmake_minimum_required(VERSION 2.8.3)
project(perch_experiment)

set(PKG_DEPENDS
  experiment_msgs
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
find_package(Curses REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS ${RUN_DEPENDS}
  # DEPENDS Curses
)

add_executable(exploration_vis src/exploration_visualizer.cpp)
target_link_libraries(exploration_vis ${catkin_LIBRARIES})

add_executable(results_vis src/results_visualizer.cpp)
target_link_libraries(results_vis ${catkin_LIBRARIES})

add_executable(commander src/commander.cpp)
target_link_libraries(commander ${catkin_LIBRARIES} ${CURSES_LIBRARIES})

add_executable(fake_traffic src/fake_traffic_node.cpp)
target_link_libraries(fake_traffic ${catkin_LIBRARIES})
