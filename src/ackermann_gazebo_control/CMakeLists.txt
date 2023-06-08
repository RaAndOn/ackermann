cmake_minimum_required(VERSION 3.16.3)
project(ackermann_gazebo_control)

## Compile as C++17
add_compile_options(-std=c++17 -Wpedantic)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  ackermann_msgs
  tf2_ros
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

catkin_package(
  # DEPENDS
  #   roscpp
  #   gazebo_ros
  #   ackermann_msgs
)

add_library(AckermannControlPlugin SHARED plugins/AckermannControlPlugin.cpp)
target_link_libraries(AckermannControlPlugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
