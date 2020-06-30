#pragma once

#include <mutex>
#include <ros/ros.h>

#include <visualization_msgs/MarkerArray.h>

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

  void initializeMarkers();
  void addMarkerToArray(visualization_msgs::MarkerArray &markerArray,
                        const State &state);

  void reset();
};
