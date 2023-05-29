#include <ros/ros.h>

#include <ackermann_controller/pure_pursuit.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "controller_node");
  ros::NodeHandle privateNH("~");
  ros::NodeHandle publicNH("");

  const PurePursuit purePursuit(privateNH, publicNH);

  ros::Rate loop_rate(20);
  ROS_INFO("Init controller node\n");
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
