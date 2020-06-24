#include <cmath>
#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <math.h>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>

#include "AckermannControlPlugin.hpp"

using namespace gazebo;

AckermannControlPlugin::AckermannControlPlugin() : ModelPlugin() {}

AckermannControlPlugin::~AckermannControlPlugin(){};

void AckermannControlPlugin::Load(physics::ModelPtr parent,
                                  sdf::ElementPtr sdf) {
  // sdf contains all the elements in the <plugin> tag in the urdf file

  // Lock thread
  std::lock_guard<std::mutex> lock(this->m_mutex);

  // Store the pointer to the model
  this->m_model = parent;
  // Store base link pointer
  this->m_baseLink = m_model->GetLink("base_link");
  // Store steering joint pointers
  this->m_flWheelSteeringJoint =
      this->m_model->GetJoint("front_left_steer_joint");
  this->m_frWheelSteeringJoint =
      this->m_model->GetJoint("front_right_steer_joint");
  // Store velocity joint pointers
  this->m_flWheelVelocityJoint =
      this->m_model->GetJoint("front_left_wheel_velocity_joint");
  this->m_frWheelVelocityJoint =
      this->m_model->GetJoint("front_right_wheel_velocity_joint");
  this->m_blWheelVelocityJoint =
      this->m_model->GetJoint("back_left_wheel_velocity_joint");
  this->m_brWheelVelocityJoint =
      this->m_model->GetJoint("back_right_wheel_velocity_joint");
  // Store world pointer
  this->m_world = this->m_model->GetWorld();

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->m_updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&AckermannControlPlugin::OnUpdate, this));

  if (sdf->HasElement("robotNamespace")) {
    this->m_robotNamespace =
        sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
  }

  if (sdf->HasElement("longitudinal_length_between_wheels")) {
    this->m_l = sdf->Get<double>("longitudinal_length_between_wheels");
  } else {
    ROS_ERROR("Parameter 'longitudinal_length_between_wheels' missing");
  }
  if (sdf->HasElement("longitudinal_length_between_wheels")) {
    this->m_w = sdf->Get<double>("lateral_width_between_wheels");
  } else {
    ROS_ERROR("Parameter 'lateral_width_between_wheels' missing");
  }

  ros::NodeHandle m_nh(this->m_robotNamespace);
  this->m_controlSub = m_nh.subscribe(
      "cmd_vel", 10, &AckermannControlPlugin::controlCallback, this);

  this->m_steeringPub = m_nh.advertise<ackermann_msgs::AckermannSteering>(
      "ackermann/steering", 1);

  this->m_groundTruthPub =
      m_nh.advertise<nav_msgs::Odometry>("ackermann/ground_truth", 1);

  this->m_jointPub =
      m_nh.advertise<sensor_msgs::JointState>("/joint_states", 1);

  // Initialize variables
  this->m_lastSimTime = 0.0;
  this->m_desiredVelocity = 0.0;
  this->m_desiredSteerAngle = 0.0;
  this->m_desiredLeftWheelAngle = 0.0;
  this->m_desiredRightWheelAngle = 0.0;

  // Get steering angle PID gains
  // Use the same gains for both front wheels
  const double wheelSteeringAnglePGain{
      sdf->Get<double>("wheel_steering_angle_p_gain")};
  const double wheelSteeringAngleIGain{
      sdf->Get<double>("wheel_steering_angle_i_gain")};
  const double wheelSteeringAngleDGain{
      sdf->Get<double>("wheel_steering_angle_d_gain")};
  // Max steering force magnitude
  // TODO: Calculate
  double maxSteeringForceMagnitude{5000.0};
  // Set front right left steering angle PID gains
  this->m_flWheelSteerAnglePID.SetPGain(wheelSteeringAnglePGain);
  this->m_flWheelSteerAnglePID.SetIGain(wheelSteeringAngleIGain);
  this->m_flWheelSteerAnglePID.SetDGain(wheelSteeringAngleDGain);
  this->m_flWheelSteerAnglePID.SetCmdMax(maxSteeringForceMagnitude);
  this->m_flWheelSteerAnglePID.SetCmdMin(-maxSteeringForceMagnitude);

  // Set front right wheel steering angle PID gains
  this->m_frWheelSteerAnglePID.SetPGain(wheelSteeringAnglePGain);
  this->m_frWheelSteerAnglePID.SetIGain(wheelSteeringAngleIGain);
  this->m_frWheelSteerAnglePID.SetDGain(wheelSteeringAngleDGain);
  this->m_frWheelSteerAnglePID.SetCmdMax(maxSteeringForceMagnitude);
  this->m_frWheelSteerAnglePID.SetCmdMin(-maxSteeringForceMagnitude);

  // Get wheel velocity angle PID gains
  // Use the same PID controller for both front wheels
  // This is an implicit differential
  const double wheelVelocityPGain{
      sdf->Get<double>("wheel_angular_velocity_p_gain")};
  const double wheelVelocityIGain{
      sdf->Get<double>("wheel_angular_velocity_i_gain")};
  const double wheelVelocityDGain{
      sdf->Get<double>("wheel_angular_velocity_d_gain")};
  // Calculate max wheel speed
  // TODO: Calculate
  double maxSpeed = sdf->Get<double>("max_speed");
  double wheelDiameter = sdf->Get<double>("wheel_diam");

  m_maxWheelSpeed = maxSpeed / (M_PI * wheelDiameter) * 10;
  // Set wheel velocity PID gains
  this->m_wheelAngularVelocityPID.SetPGain(wheelVelocityPGain);
  this->m_wheelAngularVelocityPID.SetIGain(wheelVelocityIGain);
  this->m_wheelAngularVelocityPID.SetDGain(wheelVelocityDGain);
  this->m_wheelAngularVelocityPID.SetCmdMax(m_maxWheelSpeed);
  this->m_wheelAngularVelocityPID.SetCmdMin(-m_maxWheelSpeed);
}

// Called by the world update start event
void AckermannControlPlugin::OnUpdate() {
  // Lock thread
  std::lock_guard<std::mutex> lock(this->m_mutex);

  // Calculate dt since last time step
  common::Time currTime = this->m_world->SimTime();
  double dt{(currTime - this->m_lastSimTime).Double()};
  // if (dt < 0) {
  //   this->Reset();
  //   return;
  // } else if (ignition::math::equal(dt, 0.0)) {
  //   return;
  // }

  // Initialize ground truth message
  nav_msgs::Odometry groundTruthOdom;
  groundTruthOdom.header.stamp = ros::Time::now();

  // Get current linear velocity of the vehicle
  const double direction{AckermannControlPlugin::sign(
      this->m_flWheelVelocityJoint->GetVelocity(0))};
  const auto currentLinearVelocity{m_baseLink->WorldCoGLinearVel()};
  const double currentLinearSpeed{direction * currentLinearVelocity.Length()};

  // Set the linear velocity in the ground truth  message
  groundTruthOdom.twist.twist.linear.x = currentLinearVelocity.X();
  groundTruthOdom.twist.twist.linear.y = currentLinearVelocity.Y();
  groundTruthOdom.twist.twist.linear.z = currentLinearVelocity.Z();
  // Get current angular velocity of the vehicle and set in ground truth message
  const auto currentAngularVelocity{m_baseLink->WorldAngularVel()};
  groundTruthOdom.twist.twist.angular.x = currentAngularVelocity.X();
  groundTruthOdom.twist.twist.angular.y = currentAngularVelocity.Y();
  groundTruthOdom.twist.twist.angular.z = currentAngularVelocity.Z();
  // Get current world pose
  const auto worldPose{m_baseLink->WorldCoGPose()};
  // Set current position in ground truth message
  const auto worldPos{worldPose.Pos()};
  groundTruthOdom.pose.pose.position.x = worldPos.X();
  groundTruthOdom.pose.pose.position.y = worldPos.Y();
  groundTruthOdom.pose.pose.position.z = worldPos.Z();
  // Set current orientation in ground truth message
  const auto worldRot{worldPose.Rot()};
  groundTruthOdom.pose.pose.orientation.x = worldRot.X();
  groundTruthOdom.pose.pose.orientation.y = worldRot.Y();
  groundTruthOdom.pose.pose.orientation.z = worldRot.Z();
  groundTruthOdom.pose.pose.orientation.w = worldRot.W();

  // Publish ground truth
  this->m_groundTruthPub.publish(groundTruthOdom);

  // Calculate the current error between the desired and actual vehicle
  // velocity
  const double velocityError = currentLinearSpeed - this->m_desiredVelocity;

  // Get the new wheel force command based on the linear velocity error and
  // the PID
  // TODO: Do I want to update the PID even if velocity error is less than eps?
  double wheelForceCmd =
      this->m_wheelAngularVelocityPID.Update(velocityError, dt);

  // Check that velocity error is large enough to apply a force
  if (std::abs(velocityError) > 0.1) {
    // Send the new force to both wheels
    // "0" is the index zero-indexed value of the roll position
    // TODO: Figure out what "0" means
    this->m_flWheelVelocityJoint->SetForce(0, wheelForceCmd);
    this->m_frWheelVelocityJoint->SetForce(0, wheelForceCmd);
  }

  // Get current steer angles of both wheels
  // TODO: Figure out what "0" means, I've tried "1" and "2" as well, no
  // difference
  double flWheelAngle = this->m_flWheelSteeringJoint->Position(0);
  double frWheelAngle = this->m_frWheelSteeringJoint->Position(0);

  // Calculate the current error between the desired and actual steer angle
  const double flError = flWheelAngle - this->m_desiredLeftWheelAngle;
  const double frError = frWheelAngle - this->m_desiredRightWheelAngle;

  // Calculate the new steering angle force commands
  // TODO: Do I want to update the PID even if velocity error is less than eps?
  double flWheelForceCmd = this->m_flWheelSteerAnglePID.Update(flError, dt);
  double frWheelForceCmd = this->m_frWheelSteerAnglePID.Update(frError, dt);

  if (std::abs(flError) > DBL_EPSILON) {
    // Apply the new force commands
    // TODO: Figure out what "0" means, I've tried "1" and "2" as well, no
    // difference
    this->m_flWheelSteeringJoint->SetForce(0, flWheelForceCmd);
  }

  if (std::abs(frError) > DBL_EPSILON) {
    // Apply the new force commands
    // TODO: Figure out what "0" means, I've tried "1" and "2" as well, no
    // difference
    this->m_frWheelSteeringJoint->SetForce(0, frWheelForceCmd);
  }

  // Publish updated joint states for RVIZ visualization
  sensor_msgs::JointState joint_state;
  joint_state.header.stamp = ros::Time::now();
  joint_state.name.resize(6);
  joint_state.position.resize(6);
  joint_state.name[0] = "back_left_wheel_velocity_joint";
  joint_state.position[0] = this->m_blWheelVelocityJoint->Position(0);
  joint_state.name[1] = "back_right_wheel_velocity_joint";
  joint_state.position[1] = this->m_brWheelVelocityJoint->Position(0);
  joint_state.name[2] = "front_left_steer_joint";
  joint_state.position[2] = flWheelAngle;
  joint_state.name[3] = "front_left_wheel_velocity_joint";
  joint_state.position[3] = this->m_flWheelVelocityJoint->Position(0);
  joint_state.name[4] = "front_right_steer_joint";
  joint_state.position[4] = frWheelAngle;
  joint_state.name[5] = "front_right_wheel_velocity_joint";
  joint_state.position[5] = this->m_frWheelVelocityJoint->Position(0);

  this->m_jointPub.publish(joint_state);

  // Publish ground truth pose of the base link in gazebo to TF topic
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.frame_id = "ground_truth";
  odom_trans.child_frame_id = "base_link";
  odom_trans.header.stamp = ros::Time::now();
  odom_trans.transform.translation.x = worldPos.X();
  odom_trans.transform.translation.y = worldPos.Y();
  odom_trans.transform.translation.z = worldPos.Z();
  odom_trans.transform.rotation.x = worldRot.X();
  odom_trans.transform.rotation.y = worldRot.Y();
  odom_trans.transform.rotation.z = worldRot.Z();
  odom_trans.transform.rotation.w = worldRot.W();
  this->m_tfBroadcaster.sendTransform(odom_trans);

  // Publish current steering geometry of vehicle
  ackermann_msgs::AckermannSteering currentSteering;
  currentSteering.front_left_steering_angle = flWheelAngle;
  currentSteering.front_right_steering_angle = frWheelAngle;
  currentSteering.speed = currentLinearSpeed;
  m_steeringPub.publish(currentSteering);

  // Update last sim time
  this->m_lastSimTime = currTime;
}

// Called by the world update start event
void AckermannControlPlugin::controlCallback(
    const geometry_msgs::TwistStamped &cmd) {
  // Lock thread
  std::lock_guard<std::mutex> lock(this->m_mutex);
  // linear.x represents the desired linear velocity of the vehicle (m/s)
  this->m_desiredVelocity = cmd.twist.linear.x;
  // angular.z represents the desired steering angle of the vehicle (radians)
  this->m_desiredSteerAngle = cmd.twist.angular.z;
  ROS_INFO("Desired Velocity: %s m/s",
           std::to_string(m_desiredVelocity).c_str());
  ROS_INFO("Desired Steer Angle: %s radians",
           std::to_string(m_desiredSteerAngle).c_str());

  // See `ackermann_geometry.pdf` for math
  this->m_desiredLeftWheelAngle =
      atan2(2 * this->m_l * sin(m_desiredSteerAngle),
            2 * this->m_l * cos(m_desiredSteerAngle) -
                this->m_w * sin(m_desiredSteerAngle));
  this->m_desiredRightWheelAngle =
      atan2(2 * this->m_l * sin(m_desiredSteerAngle),
            2 * this->m_l * cos(m_desiredSteerAngle) +
                this->m_w * sin(m_desiredSteerAngle));

  ROS_INFO("Desired Left Wheel Angle: %s radians",
           std::to_string(m_desiredLeftWheelAngle).c_str());
  ROS_INFO("Desired Right Wheel Angle: %s radians",
           std::to_string(m_desiredRightWheelAngle).c_str());
}

double AckermannControlPlugin::sign(const double num) const {
  if (num < 0)
    return -1.0;
  if (num > 0)
    return 1.0;
  return 0.0;
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(AckermannControlPlugin)