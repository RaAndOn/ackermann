#pragma once

// #include <gazebo/gazebo.hh>
#include <geometry_msgs/TwistStamped.h>
#include <ignition/math/Vector3.hh>
#include <mutex>
#include <ros/ros.h>

#include <ackermann_msgs/AckermannSteering.h>

// using namespace gazebo;

namespace gazebo {
// Forward declaration
// class AckermannControlPluginPrivate;

/// @brief A model plugin for controlling an ackermann vehicle model in gazebo
class AckermannControlPlugin : public ModelPlugin {
  /// @brief Constructor.
public:
  AckermannControlPlugin();

  /// @brief Destructor.
  virtual ~AckermannControlPlugin();

  // Documentation Inherited
  // public:
  //   virtual void Reset();

  /// @brief Load the controller.
  void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);

  /// @brief ROS subscriber callback
  // private:
  //   void VelocityCommandCallback(const geometry_msgs::TwistStamped &msg);

private:
  /// @brief Private data
  // private:
  //   std::unique_ptr<AckermannControlPluginPrivate> m_dataPtr;

  std::string m_robotNamespace;

  ros::NodeHandle m_nh;

  ros::Subscriber m_controlSub;

  ros::Publisher m_steeringPub;

  event::ConnectionPtr m_updateConnection;

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

  double m_l;

  double m_w;

  double m_maxWheelSpeed;

  double m_desiredVelocity;

  double m_desiredSteerAngle;

  double m_desiredLeftSteerAngle;

  double m_desiredRightSteerAngle;

  std::mutex m_mutex;

  /// @brief Update on every time step
  void OnUpdate();

  void controlCallback(const geometry_msgs::TwistStamped &cmd);

  double sign(const double num);
};
} // namespace gazebo
