cmake_minimum_required(VERSION 3.10)
project(armor_detector)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

add_definitions(-Wall -Werror)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  tf
  nodelet
  dynamic_reconfigure
  message_generation
)

generate_dynamic_reconfigure_options(
  cfg/ArmorParam.cfg
)

catkin_package(
   INCLUDE_DIRS
   include
#  LIBRARIES armor_detector
   CATKIN_DEPENDS
   cv_bridge
   image_transport
   roscpp
   sensor_msgs
   std_msgs
   tf
   nodelet
   dynamic_reconfigure
   message_generation
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}.cpp
)

add_dependencies(${PROJECT_NAME}
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

