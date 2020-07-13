#include <geometry_msgs/TwistStamped.h>

#include <ackermann_controller/pure_pursuit.hpp>

PurePursuit::PurePursuit(ros::NodeHandle &privateNH, ros::NodeHandle &publicNH)
    : m_privateNH{privateNH}, m_publicNH{publicNH} {
  m_privateNH.param<std::string>("vehicle_odom_topic", m_vehicleOdomTopic,
                                 "ground_truth");
  m_privateNH.param<std::string>("vehicle_control_topic", m_vehicleControlTopic,
                                 "cmd_vel");
  m_privateNH.param("look_ahead_distance", m_lookAheadDistance, 10.0);
  m_privateNH.param("velocity", m_velocity, 5.0);

  m_vehicleSub = m_publicNH.subscribe("m_vehicleOdomTopic", 1,
                                      &PurePursuit::controlCallback, this);

  m_controlPub = m_publicNH.advertise<geometry_msgs::TwistStamped>(
      "m_vehicleControlTopic", 1);
}

PurePursuit::~PurePursuit() = default;

void PurePursuit::controlCallback(const nav_msgs::Odometry &pose) {}