cmake_minimum_required(VERSION 2.8.12)
project(experiment_msgs)

set(PKG_DEPENDS
  std_msgs
)
set(BUILD_DEPENDS
  message_generation
  ${PKG_DEPENDS}
)
set(RUN_DEPENDS
  message_runtime
  ${PKG_DEPENDS}
)

find_package(catkin REQUIRED ${BUILD_DEPENDS})

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_message_files(
  DIRECTORY
    msg
  FILES
    Command.msg
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS ${RUN_DEPENDS}
  DEPENDS
)
