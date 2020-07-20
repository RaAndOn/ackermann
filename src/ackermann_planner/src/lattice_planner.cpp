#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <ackermann_msgs/AckermannPath.h>
#include <ackermann_planner/lattice_planner.hpp>

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
  m_pathPub =
      m_publicNH.advertise<ackermann_msgs::AckermannPath>(m_pathTopic, 1);
  m_vehicleSub = m_publicNH.subscribe(
      m_vehicleOdomTopic, 1, &LatticePlanner::updateStateCallback, this);
  m_planPathSrv =
      m_publicNH.advertiseService("plan_path", &LatticePlanner::planPath, this);
}

LatticePlanner::~LatticePlanner() = default;

void LatticePlanner::updateStateCallback(const nav_msgs::Odometry &odom) {
  std::lock_guard<std::mutex> plannerLock(m_plannerMutex);

  m_vehicleState = odom;
}

bool LatticePlanner::planPath(ackermann_planner::Goal::Request &req,
                              ackermann_planner::Goal::Response &res) {
  std::lock_guard<std::mutex> plannerLock(m_plannerMutex);

  // Initialize variables
  ackermann_msgs::AckermannPath pathMsg;
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
      ackermann_msgs::AckermannPoseStamped poseMsg;
      poseMsg.header.frame_id = m_vehicleOdomTopic;
      poseMsg.header.stamp = ros::Time::now();
      poseMsg.pose.position.x = state.m_x;
      poseMsg.pose.position.y = state.m_y;

      tf2::Quaternion quat;
      quat.setRPY(0, 0, state.m_theta);
      tf2::convert(quat, poseMsg.pose.orientation);

      poseMsg.pose.gear = state.m_gear;
      pathMsg.poses.push_back(poseMsg);
    }
  } else {
    res.success = false;
    ROS_ERROR("No path found");
  }

  // Publish path
  m_pathPub.publish(pathMsg);
  return true;
}
