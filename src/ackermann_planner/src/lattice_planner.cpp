#include <ackermann_planner/lattice_planner.hpp>

// Point x -- east, y -- north and z -- up yaw
LatticePlanner::LatticePlanner(ros::NodeHandle &privateNH,
                               ros::NodeHandle &publicNH)
    : m_privateNH{privateNH}, m_publicNH{publicNH} {
  // Get parameters from Param server
  //   privateNH_.param("kp_angular", kpAngular_, 1.0);
  //   privateNH_.param<std::string>("robot_frame", m_robotFrame, "base_link");
  //   privateNH_.param<std::string>("world_frame", m_worldFrame, "map");
  // Set publishers and subscribers
  m_pubVisualization = m_publicNH.advertise<visualization_msgs::MarkerArray>(
      "visualization_marker", 0);

  initializeMarkers();

  // Initialize loop terms
  //   setLoopTerms(nav_msgs::Path{});
  //   pubPathStatus_.publish(goalPath_);
}

LatticePlanner::~LatticePlanner() = default;

void LatticePlanner::initializeMarkers() {
  m_forwardMarker.header.frame_id = "ground_truth";
  m_forwardMarker.header.stamp = ros::Time();
  // marker.ns = "my_namespace";
  m_forwardMarker.id = 0;
  m_forwardMarker.type = visualization_msgs::Marker::ARROW;
  m_forwardMarker.action = visualization_msgs::Marker::ADD;
  m_forwardMarker.pose.position.x = 1;
  m_forwardMarker.pose.position.y = 1;
  m_forwardMarker.pose.position.z = 1;
  m_forwardMarker.pose.orientation.x = 0.0;
  m_forwardMarker.pose.orientation.y = 0.0;
  m_forwardMarker.pose.orientation.z = 0.0;
  m_forwardMarker.pose.orientation.w = 1.0;
  m_forwardMarker.scale.x = .4;
  m_forwardMarker.scale.y = 0.075;
  m_forwardMarker.scale.z = 0.075;
  m_forwardMarker.color.a = 1.0; // Don't forget to set the alpha!
  m_forwardMarker.color.r = 0.0;
  m_forwardMarker.color.g = 1.0;
  m_forwardMarker.color.b = 0.0;

  m_reverseMarker = m_forwardMarker;
  m_reverseMarker.scale.x = .3;
  m_reverseMarker.scale.y = 0.15;
  m_reverseMarker.scale.z = 0.15;
}

void LatticePlanner::visualizationLoopTEST() {
  visualization_msgs::MarkerArray markerArray;
  addMarkerToArray(markerArray, {0, 1, 1, 0, Gear::FORWARD});
  addMarkerToArray(markerArray, {1, 2, 1, .3, Gear::REVERSE});
  m_pubVisualization.publish(markerArray);
}

void LatticePlanner::addMarkerToArray(
    visualization_msgs::MarkerArray &markerArray, const State &state) {
  // Initialize marker shape based on the gear
  visualization_msgs::Marker marker;
  if (state.m_gear == Gear::FORWARD) {
    marker = m_forwardMarker;
  } else if (state.m_gear == Gear::REVERSE) {
    marker = m_reverseMarker;
  } else {
    ROS_ERROR("Member variable 'm_gear' of state was not set properly");
  }
  // Set Marker specifics
  marker.id = state.m_id;
  marker.pose.position.x = state.m_x;
  marker.pose.position.y = state.m_y;
  marker.pose.orientation.z = state.m_theta;

  // Add marker to marker array
  markerArray.markers.push_back(marker);
}

void LatticePlanner::reset() {}
