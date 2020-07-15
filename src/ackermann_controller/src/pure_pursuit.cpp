#include <climits>

#include <geometry_msgs/TwistStamped.h>
#include <tf2/utils.h>

#include <ackermann_controller/pure_pursuit.hpp>

PurePursuit::PurePursuit(ros::NodeHandle &privateNH, ros::NodeHandle &publicNH)
    : m_privateNH{privateNH}, m_publicNH{publicNH}, m_tfListener{m_tfBuffer} {
  ROS_INFO("Initialize Pure Pursuit");
  m_privateNH.param<std::string>("vehicle_odom_topic", m_vehicleOdomTopic,
                                 "ground_truth");
  m_privateNH.param<std::string>("vehicle_control_topic", m_vehicleControlTopic,
                                 "cmd_vel");
  m_privateNH.param<std::string>("vehicle_path_topic", m_pathTopic, "path");
  m_privateNH.param("look_ahead_distance", m_lookAheadDistance, 10.0);
  m_privateNH.param("velocity", m_velocity, 5.0);

  m_vehicleSub = m_publicNH.subscribe(m_vehicleOdomTopic, 1,
                                      &PurePursuit::controlCallback, this);
  m_pathSub =
      m_publicNH.subscribe(m_pathTopic, 1, &PurePursuit::pathCallback, this);

  m_controlPub = m_publicNH.advertise<geometry_msgs::TwistStamped>(
      m_vehicleControlTopic, 1);
  m_pathPub = m_publicNH.advertise<nav_msgs::Path>(m_pathTopic, 1);
}

PurePursuit::~PurePursuit() = default;

void PurePursuit::controlCallback(const nav_msgs::Odometry &odom) {
  // Update vehicle state
  std::lock_guard<std::mutex> controllerLock(m_controllerMutex);
  m_vehicleState = odom;

  // Only send control messages if there is a path to follow
  if (not m_path.empty()) {
    purePursuit();
  }
}

void PurePursuit::purePursuit() {
  // Find the closest point on the path
  int closestPointIndex = PurePursuit::findIndexOfClosestPointOnPath();
  // Remove points on the path which have been passed
  m_path.erase(m_path.begin(), m_path.begin() + closestPointIndex);

  // Clear path if one point is left
  if (m_path.size() <= 1) {
    m_path.clear();
  }

  // Publish updated path
  nav_msgs::Path updatedPath;
  updatedPath.header.frame_id = m_vehicleOdomTopic;
  updatedPath.header.stamp = ros::Time::now();
  updatedPath.poses = m_path;
  m_pathPub.publish(updatedPath);

  // Command vehicle to stop if path is empty
  if (m_path.empty()) {
    geometry_msgs::TwistStamped cmd;
    m_controlPub.publish(cmd);
    return;
  }

  // Initialize variables for finding look ahead point
  double vehicleX{m_vehicleState.pose.pose.position.x};
  double vehicleY{m_vehicleState.pose.pose.position.y};
  double poseDistance{0};

  // Initialize the look ahead pose as the last point in the path
  // This handles the situation where the end of the path is less than a look
  // ahead distance away
  geometry_msgs::PoseStamped lookAheadPoseOdom;
  lookAheadPoseOdom = m_path.back();

  // Find look ahead point
  for (auto pose : m_path) {
    poseDistance = std::sqrt(std::pow(pose.pose.position.x - vehicleX, 2.0) +
                             std::pow(pose.pose.position.y - vehicleY, 2.0));
    if (poseDistance > m_lookAheadDistance) {
      lookAheadPoseOdom = pose;
      break;
    }
  }

  // Transform the look ahead pose from the world frame to the robot frame
  geometry_msgs::TransformStamped transform;
  geometry_msgs::PoseStamped lookAheadPoseRobot;
  bool notDone{true};
  while (notDone) {
    try {
      // Get the latest transform, as indicated by ros::Time(0)
      // TODO: Decide if latest transform is best. Waiting for a transform was
      // proving too slow
      transform = m_tfBuffer.lookupTransform("base_link", "ground_truth",
                                             ros::Time(0), ros::Duration(0.0));
      tf2::doTransform(lookAheadPoseOdom, lookAheadPoseRobot, transform);
      notDone = false;
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
  }

  // Calculate steering angle based on pure pursuit algorithm
  double steeringAngle{2 * lookAheadPoseRobot.pose.position.y /
                       std::pow(poseDistance, 2.0)};

  // Publish command
  geometry_msgs::TwistStamped cmd;
  cmd.twist.linear.x = m_velocity;
  cmd.twist.angular.z = steeringAngle;
  m_controlPub.publish(cmd);
}

void PurePursuit::pathCallback(const nav_msgs::Path &path) {
  std::lock_guard<std::mutex> controllerLock(m_controllerMutex);
  m_path = path.poses;
}

int PurePursuit::findIndexOfClosestPointOnPath() {
  // Initialize vehicle pose variables
  double vehicleX{m_vehicleState.pose.pose.position.x};
  double vehicleY{m_vehicleState.pose.pose.position.y};
  double vehicleTheta{tf2::getYaw(m_vehicleState.pose.pose.orientation)};

  // Initialize variables
  auto closestPointIt = m_path.begin();
  double minDistanceToPath{__DBL_MAX__};

  // The closest point is the last point to be closer to the vehicle than the
  // previous point
  for (auto pathIt = m_path.begin(); pathIt != m_path.end(); ++pathIt) {
    auto distanceToPoint{
        std::sqrt(std::pow(pathIt->pose.position.x - vehicleX, 2.0) +
                  std::pow(pathIt->pose.position.y - vehicleY, 2.0))};
    if (distanceToPoint > minDistanceToPath) {
      break;
    }
    closestPointIt = pathIt;
    minDistanceToPath = distanceToPoint;
  }

  // Distance from the first pointer to the closest point pointer
  return std::distance(m_path.begin(), closestPointIt);
}