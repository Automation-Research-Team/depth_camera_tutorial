cmake_minimum_required(VERSION 3.8)
project(depth_camera_tutorial)

enable_language(CXX)

set(CMAKE_CXX_STANDARD		17)
set(CMAKE_CXX_STANDARD_REQUIRED	ON)
set(CMAKE_CXX_EXTENSIONS	OFF)
set(CMAKE_CXX_FLAGS_RELEASE	"-Wall -O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG	"-g")

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    roslib
    image_transport
    sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    image_transport
    sensor_msgs
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pointcloud_example
  src/pointcloud_example.cpp
)

add_dependencies(pointcloud_example
  ${${PREOJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(pointcloud_example
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
)

add_executable(depth_example
  src/depth_example.cpp
)

add_dependencies(depth_example
  ${${PREOJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(depth_example
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(color_depth_example
  src/color_depth_example.cpp
)

add_dependencies(color_depth_example
  ${${PREOJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(color_depth_example
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

#############
## Install ##
#############
install(
  TARGETS
    pointcloud_example depth_example color_depth_example
  ARCHIVE
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
