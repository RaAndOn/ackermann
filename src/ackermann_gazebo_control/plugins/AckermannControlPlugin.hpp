#pragma once

// #include <gazebo/gazebo.hh>
#include <geometry_msgs/TwistStamped.h>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>

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

  event::ConnectionPtr m_updateConnection;

  physics::ModelPtr m_model;

  /// @brief Linear velocity of chassis c.g. in world frame at last update (m/s)
  ignition::math::Vector3d m_vehicleLinearVelocity;

  /// @brief Chassis link
  physics::LinkPtr m_baseLink;

  /// @brief Update on every time step
  void OnUpdate();

  void controlCallback(const geometry_msgs::TwistStamped &cmd);
};
} // namespace gazebo
