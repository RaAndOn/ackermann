cmake_minimum_required(VERSION 3.16.3)
project(gazebo_push_plugin)

## Compile as C++17
add_compile_options(-std=c++17 -Wpedantic -fsanitize=address,undefined)
add_link_options(-fsanitize=address,undefined)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

catkin_package(
  DEPENDS
    roscpp
    gazebo_ros
)

add_library(model_push SHARED plugin/model_push.cpp)
target_link_libraries(model_push ${GAZEBO_LIBRARIES})
