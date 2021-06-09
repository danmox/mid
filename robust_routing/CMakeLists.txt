cmake_minimum_required(VERSION 2.8.12)
project(robust_routing)

set(PKG_DEPENDS
  rospy
  routing_msgs
)

set(BUILD_DEPENDS
  ${PKG_DEPENDS}
  message_generation
)

set(RUN_DEPENDS
  ${PKG_DEPENDS}
  message_runtime
)

find_package(catkin REQUIRED COMPONENTS
  ${BUILD_DEPENDS}
)

add_service_files(FILES
  CommSpec.srv
)

catkin_python_setup()

generate_messages(DEPENDENCIES std_msgs routing_msgs)

catkin_package(
  CATKIN_DEPENDS ${RUN_DEPENDS}
)

install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
