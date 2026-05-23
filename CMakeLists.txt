cmake_minimum_required(VERSION 3.0.2)
project(roscar1)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  sensor_msgs
  geometry_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)

# ROS Melodic cv_bridge on this robot is built against Ubuntu's OpenCV 3.2,
# while vendor OpenCV 3.3 may also be installed under /usr or /usr/local.
# Keep this node on the same ABI as cv_bridge when those system libs exist.
if(EXISTS "/usr/lib/aarch64-linux-gnu/libopencv_core.so.3.2.0")
  set(OpenCV_INCLUDE_DIRS
    /usr/include
    /usr/include/opencv
  )
  set(OpenCV_LIBRARIES
    /usr/lib/aarch64-linux-gnu/libopencv_core.so.3.2.0
    /usr/lib/aarch64-linux-gnu/libopencv_imgproc.so.3.2.0
    /usr/lib/aarch64-linux-gnu/libopencv_imgcodecs.so.3.2.0
  )
endif()

catkin_package(
  CATKIN_DEPENDS
    roscpp
    cv_bridge
    image_transport
    sensor_msgs
    geometry_msgs
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(line_follower_node src/line_follower.cpp)

target_link_libraries(line_follower_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

install(TARGETS line_follower_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
