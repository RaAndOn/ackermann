#pragma once

#include <mutex>
#include <ros/ros.h>

#include <visualization_msgs/MarkerArray.h>

#include <ackermann_planner/motion_primitive.hpp>

enum Gear { FORWARD, REVERSE };

struct State {
  int m_id;
  double m_x;
  double m_y;
  double m_theta;
  Gear m_gear;
  State(const int id, const double x, const double y, const double theta,
        const Gear gear)
      : m_id{id}, m_x{x}, m_y{y}, m_theta{theta}, m_gear{gear} {}
};

class LatticePlanner {
public:
  LatticePlanner(ros::NodeHandle &privateNH, ros::NodeHandle &publicNH);

  ~LatticePlanner();

  void visualizationLoopTEST();

private:
  ros::Publisher m_pubVisualization;
  ros::NodeHandle m_privateNH;
  ros::NodeHandle m_publicNH;

  visualization_msgs::Marker m_reverseMarker;
  visualization_msgs::Marker m_forwardMarker;

  std::vector<Primitive> m_motionPrimitivesVector;

  double m_wheelbase;
  double m_dt;

  int m_markerID;

  /// @brief This function initializes the reverse and forward markers so they
  /// look correct. This is because ROS is dumb sometimes and it takes like 20
  /// lines to do this for some reason
  void initializeMarkers();

  /// @brief This function adds a marker to the marker array for publication and
  /// visualization in RVIZ
  /// @param markerArray reference to the marker array
  /// @param state location and gear of the new marker being added to the marker
  /// array
  void addMarkerToArray(visualization_msgs::MarkerArray &markerArray,
                        const State &state);

  void reset();
};
