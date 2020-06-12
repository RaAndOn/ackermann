#pragma once

#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>

namespace gazebo {
// Forward declaration
class AckermannControlPluginPrivate;

/// \brief A model plugin for controlling an ackermann vehicle model in gazebo
class AckermannControlPlugin : public ModelPlugin {
  /// \brief Constructor.
public:
  AckermannControlPlugin();

  /// \brief Destructor.
public:
  virtual ~AckermannControlPlugin();

  // Documentation Inherited
public:
  virtual void Reset();

  /// \brief Load the controller.
public:
  void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);

  /// \brief ROS subscriber callback
private:
  void VelocityCommandCallback(const geometry_msgs::TwistStamped &msg);

  /// \brief Update on every time step
private:
  void Update();

  /// \brief Private data
private:
  std::unique_ptr<AckermannControlPluginPrivate> m_dataPtr;

  /// ROS Namespace
private:
  std::string m_robotNamespace;
};
} // namespace gazebo