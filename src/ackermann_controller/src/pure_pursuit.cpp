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
                                      &PurePursuit::updateStateCallback, this);
  m_pathSub =
      m_publicNH.subscribe(m_pathTopic, 1, &PurePursuit::pathCallback, this);

  m_controlPub = m_publicNH.advertise<geometry_msgs::TwistStamped>(
      m_vehicleControlTopic, 1);
}

PurePursuit::~PurePursuit() = default;

void PurePursuit::controlCallback(const nav_msgs::Odometry &odom) {}

void PurePursuit::updateStateCallback(const nav_msgs::Odometry &odom) {
  std::lock_guard<std::mutex> controllerLock(m_controllerMutex);
  m_vehicleState = odom;
}

void PurePursuit::pathCallback(const nav_msgs::Path &path) {}

nav_msgs::Odometry findClosestPointOnPath(const nav_msgs::Path &path) {}