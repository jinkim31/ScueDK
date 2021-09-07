cmake_minimum_required(VERSION 3.0.2)
project(scuedk)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  serial
)
catkin_package(
  INCLUDE_DIRS scuedk_ros/inc
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_executable(serial_line
                serial_line/src/main.cpp
                serial_line/src/node.cpp
                serial_line/src/packetgenerator.cpp
                serial_line/src/packettranslator.cpp
                serial_line/src/Util.cpp
                )
add_library(${PROJECT_NAME}
  scuedk_ros/src/scuedk.cpp
  structs/structs.c
)
target_link_libraries(serial_line ${catkin_LIBRARIES} serial)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
