#include <ackermann_visualization/ackermann_visualization.hpp>
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "visualization_node");
  ros::NodeHandle privateNH("~");
  ros::NodeHandle publicNH("");

  AckermannVisualization visualization(privateNH, publicNH);

  ros::Rate loop_rate(20);
  ROS_INFO("Init visualization node\n");
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}