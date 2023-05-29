cmake_minimum_required(VERSION 3.10.2)
project(ackermann_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs)
# We want boost/format.hpp, which isn't in its own component.
find_package(Boost REQUIRED)

add_message_files(
  DIRECTORY msg
  FILES AckermannSteering.msg AckermannSteeringStamped.msg AckermannPose.msg AckermannPoseStamped.msg AckermannPath.msg)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
  DEPENDS Boost
  )
