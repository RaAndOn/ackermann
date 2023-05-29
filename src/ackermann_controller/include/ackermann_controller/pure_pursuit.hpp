#ifndef ACKERMANN_CONTROLLER_PURE_PURSUIT_HPP
#define ACKERMANN_CONTROLLER_PURE_PURSUIT_HPP

#include <mutex>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include "ackermann_msgs/AckermannPath.h"

class PurePursuit {
public:
  PurePursuit(ros::NodeHandle &privateNH, ros::NodeHandle &publicNH);

private:
  /// @brief Private ROS Node Handle
  ros::NodeHandle m_privateNH;
  /// @brief Public ROS Node Handle
  ros::NodeHandle m_publicNH;

  /// @brief Subscriber to the vehicle state topic
  ros::Subscriber m_vehicleSub;
  /// @brief Subscriber to the path publication topic
  ros::Subscriber m_pathSub;

  /// @brief Publisher to the vehicle control topic
  ros::Publisher m_controlPub;
  /// @brief Publisher to the path publication topic
  ros::Publisher m_pathPub;

  /// @brief TF Buffer
  tf2_ros::Buffer m_tfBuffer;
  /// @brief TF Listener
  tf2_ros::TransformListener m_tfListener;

  /// @brief Mutex to lock threads when variables are being used
  std::mutex m_controllerMutex;

  /// @brief Variable holds the latest vehicle state
  nav_msgs::Odometry m_vehicleState;
  /// @brief Variable holds the path being followed
  std::vector<ackermann_msgs::AckermannPoseStamped> m_path;

  /// @brief Lookahead Distance (meters)
  double m_lookaheadDistance{};
  /// @brief Velocity of the vehicle
  double m_velocity{};

  /// @brief Name of the topic which publishes the vehicle state
  std::string m_vehicleOdomTopic;
  /// @brief Name of the topic which takes the vehicle control
  std::string m_vehicleControlTopic;
  /// @brief Name of the topic which publishes the vehicle path
  std::string m_pathTopic;

  /// @brief Updates the vehicle state and provides a vehicle control command if
  /// there is a path to follow.
  /// @param odom The latest vehicle state
  void controlCallback(const nav_msgs::Odometry &odom);

  /// @brief executes the Pure Pursuit algorithm as detailed in
  /// `pure_pursuit.pdf`
  void purePursuit();

  /// @brief Updates the path variable
  /// @param path The latest path
  void pathCallback(const ackermann_msgs::AckermannPath &path);

  /// @brief Find the index of the point on the path closest to the vehicle
  /// @return The index of the closest point to the vehicle
  int findIndexOfClosestPointOnPath();
};

#endif // ACKERMANN_CONTROLLER_PURE_PURSUIT_HPP
