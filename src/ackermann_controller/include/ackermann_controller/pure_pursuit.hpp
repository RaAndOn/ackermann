#pragma once

#include <mutex>

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

class PurePursuit {
public:
  PurePursuit(ros::NodeHandle &privateNH, ros::NodeHandle &publicNH);

  ~PurePursuit();

private:
  ros::NodeHandle m_privateNH;
  ros::NodeHandle m_publicNH;

  ros::Subscriber m_vehicleSub;
  ros::Subscriber m_pathSub;

  ros::Publisher m_controlPub;
  ros::Publisher m_pathPub;

  std::mutex m_controllerMutex;

  nav_msgs::Odometry m_vehicleState;
  nav_msgs::Path m_path;

  double m_lookAheadDistance;
  double m_velocity;

  std::string m_vehicleOdomTopic;
  std::string m_vehicleControlTopic;
  std::string m_pathTopic;

  void controlCallback(const nav_msgs::Odometry &odom);

  void pathCallback(const nav_msgs::Path &path);

  std::vector<geometry_msgs::PoseStamped>::iterator findClosestPointOnPath();
};