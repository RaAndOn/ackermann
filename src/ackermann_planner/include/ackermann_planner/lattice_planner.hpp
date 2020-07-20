#pragma once

#include <mutex>
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <ackermann_planner/Goal.h>
#include <ackermann_planner/a_star.hpp>
#include <ackermann_planner/motion_primitive.hpp>
#include <ackermann_project/ackermann_utils.hpp>

class LatticePlanner {
public:
  LatticePlanner(ros::NodeHandle &privateNH, ros::NodeHandle &publicNH);

  ~LatticePlanner();

private:
  /// @brief Private ROS Node Handle
  ros::NodeHandle m_privateNH;
  /// @brief Public ROS Node Handle
  ros::NodeHandle m_publicNH;

  /// @brief Publisher of path
  ros::Publisher m_pathPub;

  /// @brief Subscriber to the vehicle state topic
  ros::Subscriber m_vehicleSub;

  /// @brief Service to request a path
  ros::ServiceServer m_planPathSrv;

  /// @brief Wheelbase of vehicle is the distance between front and rear wheels
  const double m_wheelbase;

  /// @brief Time step between planning nodes
  const double m_dt;

  /// @brief Size of steering Increments to each side when calculating motion
  /// primitives
  const double m_discretizationDegrees;

  /// @brief Assumed velocity of vehicle when planning motion primitives
  const double m_velocity;

  /// @brief Number of steering increments to each side when calculating motion
  /// primitives
  const int m_steeringIncrements;

  /// @brief Angular threshold for determining if states are the same (degrees)
  const double m_angularThresholdDegrees;

  /// @brief Linear threshold for determining if states are the same (meters)
  const double m_distanceThreshold;

  /// @brief Weight on the heuristic function for an A* planner
  const double m_epsilon;

  /// @brief Name of the heuristic function
  const std::string m_heuristicFunction;

  /// @brief Name of the edge cost function
  const std::string m_edgeCostFunction;

  /// @brief Class which calculates the motion primitives of a vehicle
  const MotionPrimitive m_motionPrimitiveCalc;

  /// @brief Vector of the primitives which are used for node expansion
  const std::vector<Primitive> m_motionPrimitivesVector;

  /// @brief Linear resolution for discretizing state into integers (meters)
  const double m_distanceResolution;

  /// @brief Angular resolution for discretizing state into integers (radians)
  const double m_angularResolutionDegrees;

  /// @brief Class for searching space for a path
  AStar m_search;

  /// @brief Name of the topic which publishes the vehicle path
  const std::string m_pathTopic;

  /// @brief Name of the topic which publishes the vehicle state
  const std::string m_vehicleOdomTopic;

  /// @brief Mutex to lock threads when variables are being used
  std::mutex m_plannerMutex;

  /// @brief Variable holds the latest vehicle state
  nav_msgs::Odometry m_vehicleState;

  /// @brief Service call to find a path form the vehicle's current state to a
  /// goal state
  bool planPath(ackermann_planner::Goal::Request &req,
                ackermann_planner::Goal::Response &res);

  /// @brief Updates the vehicle state.
  /// @param odom The latest vehicle state
  void updateStateCallback(const nav_msgs::Odometry &odom);
};
