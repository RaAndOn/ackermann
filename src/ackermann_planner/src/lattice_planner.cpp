#include <nav_msgs/Path.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <ackermann_planner/lattice_planner.hpp>
#include <ackermann_project/ackermann_utils.hpp>

LatticePlanner::LatticePlanner(ros::NodeHandle &privateNH,
                               ros::NodeHandle &publicNH)
    : m_privateNH{privateNH}, m_publicNH{publicNH},
      // Instantiate Motion Primitive Parameters
      m_wheelbase{getROSParam(m_privateNH, "wheelbase", 3.0)},
      m_discretizationDegrees{
          getROSParam(m_privateNH, "discretization_degrees", 22.5)},
      m_dt{getROSParam(m_privateNH, "dt", .5)},
      m_steeringIncrements{getROSParam(m_privateNH, "steering_increments", 1)},
      m_velocity{getROSParam(m_privateNH, "velocity", 1.0)},
      // Instantiate Motion Primitives
      m_motionPrimitiveCalc{m_wheelbase, m_velocity, m_dt,
                            m_discretizationDegrees, m_steeringIncrements},
      // Insantiate Search Parameters
      m_motionPrimitivesVector{m_motionPrimitiveCalc.getMotionPrimitives()},
      m_distanceResolution{m_motionPrimitiveCalc.getDistanceResolution()},
      m_angularResolutionDegrees{m_motionPrimitiveCalc.getAngularResolution() *
                                 180 / M_PI},
      m_angularThresholdDegrees{
          getROSParam(m_privateNH, "angular_threshold_degrees", 15.0)},
      m_distanceThreshold{
          getROSParam(m_privateNH, "distance_threshold_meters", 1.0)},
      m_epsilon{getROSParam(m_privateNH, "epsilon", 1.0)},
      m_heuristicFunction{
          getROSParamString(m_privateNH, "heuristic", "Euclidean")},
      m_edgeCostFunction{
          getROSParamString(m_privateNH, "edge_cost_function", "ground_truth")},
      // Instantiate Search Method
      m_search{m_motionPrimitivesVector,   m_distanceThreshold,
               m_angularThresholdDegrees,  m_distanceResolution,
               m_angularResolutionDegrees, m_epsilon,
               m_heuristicFunction,        m_edgeCostFunction},
      // Instantiate Topic Names
      m_pathTopic{getROSParamString(m_privateNH, "vehicle_path_topic", "path")},
      m_vehicleOdomTopic{getROSParamString(m_privateNH, "vehicle_odom_topic",
                                           "ground_truth")} {

  // Set publishers and subscribers and services
  m_visualizationPub = m_publicNH.advertise<visualization_msgs::MarkerArray>(
      "visualization_marker", 1);
  m_pathPub = m_publicNH.advertise<nav_msgs::Path>(m_pathTopic, 1);
  m_vehicleSub = m_publicNH.subscribe(
      m_vehicleOdomTopic, 1, &LatticePlanner::updateStateCallback, this);
  m_planPathSrv =
      m_publicNH.advertiseService("plan_path", &LatticePlanner::planPath, this);

  initializeMarkers();
}

LatticePlanner::~LatticePlanner() = default;

void LatticePlanner::updateStateCallback(const nav_msgs::Odometry &odom) {
  std::lock_guard<std::mutex> plannerLock(m_plannerMutex);

  m_vehicleState = odom;
}

void LatticePlanner::initializeMarkers() {
  m_forwardMarker.header.frame_id = m_vehicleOdomTopic;
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

  m_stopMarker = m_forwardMarker;
  m_stopMarker.type = visualization_msgs::Marker::SPHERE;
  m_stopMarker.scale.x = 0.3;
  m_stopMarker.scale.y = 0.3;
  m_stopMarker.scale.z = 0.3;
}

bool LatticePlanner::planPath(ackermann_planner::Goal::Request &req,
                              ackermann_planner::Goal::Response &res) {
  std::lock_guard<std::mutex> plannerLock(m_plannerMutex);

  // Initialize variables
  visualization_msgs::MarkerArray markerArray;
  nav_msgs::Path pathMsg;
  pathMsg.header.frame_id = m_vehicleOdomTopic;
  pathMsg.header.stamp = ros::Time::now();

  double startX{m_vehicleState.pose.pose.position.x};
  double startY{m_vehicleState.pose.pose.position.y};
  double startTheta{tf2::getYaw(m_vehicleState.pose.pose.orientation)};
  State startState{startX, startY, startTheta, Gear::STOP};

  State goalState{req.x, req.y, req.thetaDegrees * M_PI / 180, Gear::STOP};

  // Find path to goal
  auto path = m_search.search(startState, goalState);

  // Full marker array and path with states
  if (path) {
    res.success = true;
    for (const auto &state : path.get()) {
      auto pose = addMarkerToArray(markerArray, state);
      pathMsg.poses.push_back(pose);
    }
  } else {
    res.success = false;
    ROS_ERROR("No path found");
  }

  // Publish path
  m_visualizationPub.publish(markerArray);
  m_pathPub.publish(pathMsg);
  return true;
}

geometry_msgs::PoseStamped
LatticePlanner::addMarkerToArray(visualization_msgs::MarkerArray &markerArray,
                                 const State &state) {
  // Initialize marker shape based on the gear
  visualization_msgs::Marker marker;
  if (state.m_gear == Gear::FORWARD) {
    marker = m_forwardMarker;
  } else if (state.m_gear == Gear::REVERSE) {
    marker = m_reverseMarker;
  } else if (state.m_gear == Gear::STOP) {
    marker = m_stopMarker;
  } else {
    ROS_ERROR("Member variable 'm_gear' of state was not set properly");
  }
  // Set Marker specifics
  marker.id = markerArray.markers.size();
  marker.pose.position.x = state.m_x;
  marker.pose.position.y = state.m_y;

  tf2::Quaternion quat;
  quat.setRPY(0, 0, state.m_theta);
  tf2::convert(quat, marker.pose.orientation);

  // Add marker to marker array
  markerArray.markers.push_back(marker);

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = m_vehicleOdomTopic;
  pose.header.stamp = ros::Time::now();
  pose.pose = marker.pose;
  return pose;
}
