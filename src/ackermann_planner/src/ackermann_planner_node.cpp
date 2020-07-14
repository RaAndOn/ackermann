#include <ackermann_planner/lattice_planner.hpp>
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "planner_node");
  ros::NodeHandle privateNH("~");
  ros::NodeHandle publicNH("");

  LatticePlanner latticePlanner(privateNH, publicNH);

  ros::Rate loop_rate(20);
  ROS_INFO("Init planner node\n");
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}