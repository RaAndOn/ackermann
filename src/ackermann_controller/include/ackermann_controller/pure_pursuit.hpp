#pragma once

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

class PurePursuit {
public:
  PurePursuit(ros::NodeHandle &privateNH, ros::NodeHandle &publicNH);

  ~PurePursuit();

private:
  ros::NodeHandle m_privateNH;
  ros::NodeHandle m_publicNH;

  ros::Subscriber m_vehicleSub;
  ros::Publisher m_controlPub;

  double m_lookAheadDistance;
  double m_velocity;

  std::string m_vehicleOdomTopic;
  std::string m_vehicleControlTopic;

  void controlCallback(const nav_msgs::Odometry &pose);
};