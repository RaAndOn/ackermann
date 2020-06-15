#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>

#include "AckermannControlPlugin.hpp"

using namespace gazebo;

AckermannControlPlugin::AckermannControlPlugin() : ModelPlugin() {}

AckermannControlPlugin::~AckermannControlPlugin(){};

void AckermannControlPlugin::Load(physics::ModelPtr parent,
                                  sdf::ElementPtr sdf) {
  // Store the pointer to the model
  this->m_model = parent;

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->m_updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&AckermannControlPlugin::OnUpdate, this));

  if (sdf->HasElement("robotNamespace")) {
    this->m_robotNamespace =
        sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
  }
  ros::NodeHandle m_nh(this->m_robotNamespace);
  this->m_controlSub = m_nh.subscribe(
      "cmd_vel", 10, &AckermannControlPlugin::controlCallback, this);

  std::string baseLinkName{m_model->GetName() +
                           "::" + sdf->Get<std::string>("base_link")};
  ROS_INFO("%s\n", baseLinkName.c_str());
  this->m_baseLink = m_model->GetLink(baseLinkName);
  if (!this->m_baseLink) {
    std::cerr << "could not find 'base link'" << std::endl;
    return;
  }
}

// Called by the world update start event
void AckermannControlPlugin::OnUpdate() {
  // ROS_INFO("GO!");
  this->m_model->GetLink("front_left_wheel")->SetAngularVel({0, 1, 0});
  this->m_vehicleLinearVelocity = m_baseLink->WorldCoGLinearVel();
  ROS_INFO("%s\n",
           std::to_string(this->m_vehicleLinearVelocity.Length()).c_str());
}

// Called by the world update start event
void AckermannControlPlugin::controlCallback(
    const geometry_msgs::TwistStamped &cmd) {
  ROS_INFO("Control");
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(AckermannControlPlugin)