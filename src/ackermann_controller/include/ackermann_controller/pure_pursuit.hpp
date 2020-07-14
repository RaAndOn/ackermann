#pragma once

#include <boost/optional.hpp>
#include <mutex>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

class PurePursuit {
public:
  PurePursuit(ros::NodeHandle &privateNH, ros::NodeHandle &publicNH);

  ~PurePursuit();

private:
  tf2_ros::Buffer m_tfBuffer;
  tf2_ros::TransformListener m_tfListener;

  ros::NodeHandle m_privateNH;
  ros::NodeHandle m_publicNH;

  ros::Subscriber m_vehicleSub;
  ros::Subscriber m_pathSub;

  ros::Publisher m_controlPub;
  ros::Publisher m_pathPub;

  std::mutex m_controllerMutex;

  nav_msgs::Odometry m_vehicleState;
  boost::optional<nav_msgs::Path> m_path;

  double m_lookAheadDistance;
  double m_velocity;

  std::string m_vehicleOdomTopic;
  std::string m_vehicleControlTopic;
  std::string m_pathTopic;

  void controlCallback(const nav_msgs::Odometry &odom);

  void pathCallback(const nav_msgs::Path &path);

  bool findClosestPointOnPath();
};