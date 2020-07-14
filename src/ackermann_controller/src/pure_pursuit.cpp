#include <geometry_msgs/TwistStamped.h>
#include <tf2/utils.h>

#include <ackermann_controller/pure_pursuit.hpp>

PurePursuit::PurePursuit(ros::NodeHandle &privateNH, ros::NodeHandle &publicNH)
    : m_privateNH{privateNH}, m_publicNH{publicNH} {
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
  std::lock_guard<std::mutex> controllerLock(m_controllerMutex);
  m_vehicleState = odom;

  auto currentPose = PurePursuit::findClosestPointOnPath();

  for (auto pathIt = m_path.poses.begin(); pathIt != currentPose; ++pathIt) {
    m_path.poses.erase(pathIt);
  }
  m_pathPub.publish(m_path);
}

void PurePursuit::pathCallback(const nav_msgs::Path &path) {
  std::lock_guard<std::mutex> controllerLock(m_controllerMutex);
  m_path = path;
}

std::vector<geometry_msgs::PoseStamped>::iterator
PurePursuit::findClosestPointOnPath() {
  double startX{m_vehicleState.pose.pose.position.x};
  double startY{m_vehicleState.pose.pose.position.y};
  double startTheta{tf2::getYaw(m_vehicleState.pose.pose.orientation)};

  return m_path.poses.begin();
}