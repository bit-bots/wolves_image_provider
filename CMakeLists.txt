cmake_minimum_required(VERSION 2.8.3)
project(wolves_image_provider_v4l)

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
  LIBRARIES wolves_image_provider_v4l
  CATKIN_DEPENDS camera_info_manager cv_bridge opencv2 roscpp sensor_msgs std_msgs image_transport
  DEPENDS system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(${CURSES_INCLUDE_DIR})

## Declare a cpp executable
 add_executable(wolves_image_provider_v4l_node src/main.cpp src/CaptureV4L2.cpp )

## Specify libraries to link a library or executable target against
 target_link_libraries(wolves_image_provider_v4l_node
   ${catkin_LIBRARIES}
 )
