#pragma once

// #include <gazebo/gazebo.hh>
#include <geometry_msgs/TwistStamped.h>
#include <ignition/math/Vector3.hh>
#include <mutex>
#include <ros/ros.h>

#include <tf2_ros/transform_broadcaster.h>

#include <ackermann_msgs/AckermannSteering.h>

// using namespace gazebo;

namespace gazebo {
// Forward declaration

/// @brief A model plugin for controlling an ackermann vehicle model in gazebo
class AckermannControlPlugin : public ModelPlugin {
  /// @brief Constructor.
public:
  AckermannControlPlugin();

  /// @brief Destructor.
  virtual ~AckermannControlPlugin();

  // virtual void Reset();

  /// @brief Load the controller.
  void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);

private:
  /// @brief
  std::string m_robotNamespace;

  /// @brief ROS node handle
  ros::NodeHandle m_nh;

  /// @brief ROS subscriber to listen for heading and velocity commands
  ros::Subscriber m_controlSub;

  /// @brief ROS publisher of steering joints
  ros::Publisher m_steeringPub;

  /// @brief ROS publisher of ground truth pose and velocity of base link
  ros::Publisher m_groundTruthPub;

  /// @brief ROS publisher of joints for robot_state_publisher and RVIZ
  ros::Publisher m_jointPub;

  /// @brief ROS TF broadcaster for RVIZ visualization
  tf2_ros::TransformBroadcaster m_tfBroadcaster;

  /// @brief Gazebo ptr to hold check for world update
  event::ConnectionPtr m_updateConnection;

  /// @brief Pointer to ackermann model
  physics::ModelPtr m_model;

  /// @brief Pointer to the world
  physics::WorldPtr m_world;

  /// @brief Last sim time received
  common::Time m_lastSimTime;

  /// @brief Chassis link
  physics::LinkPtr m_baseLink;

  /// @brief Front left steering joint
  physics::JointPtr m_flWheelSteeringJoint;

  /// @brief Front right steering joint
  physics::JointPtr m_frWheelSteeringJoint;

  /// @brief Front left velocity joint
  physics::JointPtr m_flWheelVelocityJoint;

  /// @brief Front right velocity joint
  physics::JointPtr m_frWheelVelocityJoint;

  /// @brief PID control for the wheel angular velocities
  common::PID m_wheelAngularVelocityPID;

  /// @brief PID control for the front left wheel steer angle
  common::PID m_flWheelSteerAnglePID;

  /// @brief PID control for the front right wheel steer angle
  common::PID m_frWheelSteerAnglePID;

  /// @brief Length between front and rear wheels in meters (Ackermann geometry)
  double m_l;

  /// @brief With between left and right wheels in meters (Ackermann geometry)
  double m_w;

  /// @brief Maximum wheel angular speed
  double m_maxWheelSpeed;

  /// @brief Desired vehicle velocity
  double m_desiredVelocity;

  /// @brief Desired steer angle of the vehicle
  double m_desiredSteerAngle;

  /// @brief Desired left wheel angle of the vehicle
  double m_desiredLeftWheelAngle;

  /// @brief Desired right wheel angle of the vehicle
  double m_desiredRightWheelAngle;

  /// @brief Mutex to prevent thread collision
  std::mutex m_mutex;

  /// @brief Update on every time step
  void OnUpdate();

  /// @brief Callback to update the desired velocity and steer angle
  /// @param cmd is the new desired velocity and steer angle
  void controlCallback(const geometry_msgs::TwistStamped &cmd);

  /// @brief Function to get the sign of a number
  /// @param num is the number whose sign we want
  /// @return the sign of num
  double sign(const double num);
};
} // namespace gazebo
