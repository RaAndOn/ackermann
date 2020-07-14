#pragma once

#include <mutex>
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <ackermann_planner/Goal.h>
#include <ackermann_planner/a_star.hpp>
#include <ackermann_planner/motion_primitive.hpp>
#include <ackermann_planner/planner_utils.hpp>

class LatticePlanner {
public:
  LatticePlanner(ros::NodeHandle &privateNH, ros::NodeHandle &publicNH);

  ~LatticePlanner();

private:
  ros::NodeHandle m_privateNH;
  ros::NodeHandle m_publicNH;

  ros::Publisher m_visualizationPub;
  ros::Publisher m_pathPub;

  ros::ServiceServer m_planPathSrv;

  visualization_msgs::Marker m_reverseMarker;
  visualization_msgs::Marker m_forwardMarker;

  std::vector<Primitive> m_motionPrimitivesVector;

  double m_wheelbase;
  double m_dt;
  double m_discretizationDegrees;
  double m_velocity;
  int m_steeringIncrements;
  double m_distanceResolution;
  double m_angularResolutionDegrees;
  double m_angularThresholdDegrees;
  double m_distanceThreshold;

  std::string m_pathTopic;

  /// @brief This function initializes the reverse and forward markers so they
  /// look correct. This is because ROS is dumb sometimes and it takes like 20
  /// lines to do this for some reason
  void initializeMarkers();

  /// @brief This function adds a marker to the marker array for publication and
  /// visualization in RVIZ
  /// @param markerArray reference to the marker array
  /// @param state location and gear of the new marker being added to the marker
  /// array
  geometry_msgs::PoseStamped
  addMarkerToArray(visualization_msgs::MarkerArray &markerArray,
                   const State &state);

  bool planPath(ackermann_planner::Goal::Request &req,
                ackermann_planner::Goal::Response &res);
};
