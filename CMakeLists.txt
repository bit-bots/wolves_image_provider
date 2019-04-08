cmake_minimum_required(VERSION 2.8.3)
project(wolves_image_provider)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  sensor_msgs
  std_msgs
  image_transport
  camera_info_manager
)

FIND_PACKAGE (OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES wolves_image_provider
  CATKIN_DEPENDS camera_info_manager cv_bridge roscpp sensor_msgs std_msgs image_transport
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(wolves_image_provider_node src/main.cpp src/CaptureV4L2.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(wolves_image_provider_node
   ${catkin_LIBRARIES}
 )
