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
  std::lock_guard<std::mutex> lock(m_mutex);

  // Store the pointer to the model
  m_model = parent;
  // Store base link pointer
  m_baseLink = m_model->GetLink("base_link");
  // Store steering joint pointers
  m_flWheelSteeringJoint = m_model->GetJoint("front_left_steer_joint");
  m_frWheelSteeringJoint = m_model->GetJoint("front_right_steer_joint");
  // Store velocity joint pointers
  m_flWheelVelocityJoint = m_model->GetJoint("front_left_wheel_velocity_joint");
  m_frWheelVelocityJoint =
      m_model->GetJoint("front_right_wheel_velocity_joint");
  m_blWheelVelocityJoint = m_model->GetJoint("back_left_wheel_velocity_joint");
  m_brWheelVelocityJoint = m_model->GetJoint("back_right_wheel_velocity_joint");
  // Store world pointer
  m_world = m_model->GetWorld();

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  m_updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&AckermannControlPlugin::OnUpdate, this));

  if (sdf->HasElement("robotNamespace")) {
    m_robotNamespace =
        sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
  }

  if (sdf->HasElement("longitudinal_length_between_wheels")) {
    m_l = sdf->Get<double>("longitudinal_length_between_wheels");
  } else {
    ROS_ERROR("Parameter 'longitudinal_length_between_wheels' missing");
  }
  if (sdf->HasElement("longitudinal_length_between_wheels")) {
    m_w = sdf->Get<double>("lateral_width_between_wheels");
  } else {
    ROS_ERROR("Parameter 'lateral_width_between_wheels' missing");
  }
  double wheelRadius;
  if (sdf->HasElement("wheel_radius")) {
    wheelRadius = sdf->Get<double>("wheel_radius");
  } else {
    ROS_ERROR("Parameter 'wheel_radius' missing");
  }
  double chassisHeight;
  if (sdf->HasElement("chassis_height")) {
    chassisHeight = sdf->Get<double>("chassis_height");
  } else {
    ROS_ERROR("Parameter 'chassis_height' missing");
  }

  std::string vehicleOdomTopic;
  if (sdf->HasElement("vehicle_odom_topic")) {
    vehicleOdomTopic = sdf->Get<std::string>("vehicle_odom_topic");
  } else {
    ROS_ERROR("Parameter 'vehicle_odom_topic' missing");
  }

  // Define Ground Truth Origin for tf broadcast and RVIZ visualization
  m_groundTruthOrigin = m_baseLink->WorldCoGPose().Pos();
  m_groundTruthOrigin.Z() = chassisHeight - 2 * wheelRadius;

  ros::NodeHandle m_nh(m_robotNamespace);
  m_controlSub = m_nh.subscribe("cmd_vel", 10,
                                &AckermannControlPlugin::controlCallback, this);

  m_steeringPub = m_nh.advertise<ackermann_msgs::AckermannSteering>(
      "ackermann/steering", 1);

  m_groundTruthPub = m_nh.advertise<nav_msgs::Odometry>(vehicleOdomTopic, 1);

  m_jointPub = m_nh.advertise<sensor_msgs::JointState>("/joint_states", 1);

  // Initialize variables
  m_lastSimTime = 0.0;
  m_desiredVelocity = 0.0;
  m_desiredSteerAngle = 0.0;
  m_desiredLeftWheelAngle = 0.0;
  m_desiredRightWheelAngle = 0.0;

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
  m_flWheelSteerAnglePID.SetPGain(wheelSteeringAnglePGain);
  m_flWheelSteerAnglePID.SetIGain(wheelSteeringAngleIGain);
  m_flWheelSteerAnglePID.SetDGain(wheelSteeringAngleDGain);
  m_flWheelSteerAnglePID.SetCmdMax(maxSteeringForceMagnitude);
  m_flWheelSteerAnglePID.SetCmdMin(-maxSteeringForceMagnitude);

  // Set front right wheel steering angle PID gains
  m_frWheelSteerAnglePID.SetPGain(wheelSteeringAnglePGain);
  m_frWheelSteerAnglePID.SetIGain(wheelSteeringAngleIGain);
  m_frWheelSteerAnglePID.SetDGain(wheelSteeringAngleDGain);
  m_frWheelSteerAnglePID.SetCmdMax(maxSteeringForceMagnitude);
  m_frWheelSteerAnglePID.SetCmdMin(-maxSteeringForceMagnitude);

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
  m_maxWheelSpeed = maxSpeed / (2 * M_PI * wheelRadius) * 10;

  // Set wheel velocity PID gains
  m_wheelAngularVelocityPID.SetPGain(wheelVelocityPGain);
  m_wheelAngularVelocityPID.SetIGain(wheelVelocityIGain);
  m_wheelAngularVelocityPID.SetDGain(wheelVelocityDGain);
  m_wheelAngularVelocityPID.SetCmdMax(m_maxWheelSpeed);
  m_wheelAngularVelocityPID.SetCmdMin(-m_maxWheelSpeed);
}

// Called by the world update start event
void AckermannControlPlugin::OnUpdate() {
  // Lock thread
  std::lock_guard<std::mutex> lock(m_mutex);

  // Calculate dt since last time step
  common::Time currTime = m_world->SimTime();
  double dt{(currTime - m_lastSimTime).Double()};
  // if (dt < 0) {
  //   Reset();
  //   return;
  // } else if (ignition::math::equal(dt, 0.0)) {
  //   return;
  // }

  // Initialize ground truth message
  nav_msgs::Odometry groundTruthOdom;
  groundTruthOdom.header.stamp = ros::Time::now();

  // Get current linear velocity of the vehicle
  const double direction{
      AckermannControlPlugin::sign(m_flWheelVelocityJoint->GetVelocity(0))};
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
  m_groundTruthPub.publish(groundTruthOdom);

  // Calculate the current error between the desired and actual vehicle
  // velocity
  const double velocityError = currentLinearSpeed - m_desiredVelocity;

  // Get the new wheel force command based on the linear velocity error and
  // the PID
  // TODO: Do I want to update the PID even if velocity error is less than eps?
  double wheelForceCmd = m_wheelAngularVelocityPID.Update(velocityError, dt);

  // Check that velocity error is large enough to apply a force
  if (std::abs(velocityError) > 0.1) {
    // Send the new force to both wheels
    // "0" is the index zero-indexed value of the roll position
    // TODO: Figure out what "0" means
    m_flWheelVelocityJoint->SetForce(0, wheelForceCmd);
    m_frWheelVelocityJoint->SetForce(0, wheelForceCmd);
  }

  // Get current steer angles of both wheels
  // TODO: Figure out what "0" means, I've tried "1" and "2" as well, no
  // difference
  double flWheelAngle = m_flWheelSteeringJoint->Position(0);
  double frWheelAngle = m_frWheelSteeringJoint->Position(0);

  // Calculate the current error between the desired and actual steer angle
  const double flError = flWheelAngle - m_desiredLeftWheelAngle;
  const double frError = frWheelAngle - m_desiredRightWheelAngle;

  // Calculate the new steering angle force commands
  // TODO: Do I want to update the PID even if velocity error is less than eps?
  double flWheelForceCmd = m_flWheelSteerAnglePID.Update(flError, dt);
  double frWheelForceCmd = m_frWheelSteerAnglePID.Update(frError, dt);

  if (std::abs(flError) > DBL_EPSILON) {
    // Apply the new force commands
    // TODO: Figure out what "0" means, I've tried "1" and "2" as well, no
    // difference
    m_flWheelSteeringJoint->SetForce(0, flWheelForceCmd);
  }

  if (std::abs(frError) > DBL_EPSILON) {
    // Apply the new force commands
    // TODO: Figure out what "0" means, I've tried "1" and "2" as well, no
    // difference
    m_frWheelSteeringJoint->SetForce(0, frWheelForceCmd);
  }

  // Publish updated joint states for RVIZ visualization
  sensor_msgs::JointState joint_state;
  joint_state.header.stamp = ros::Time::now();
  joint_state.name.resize(6);
  joint_state.position.resize(6);
  joint_state.name[0] = "back_left_wheel_velocity_joint";
  joint_state.position[0] = m_blWheelVelocityJoint->Position(0);
  joint_state.name[1] = "back_right_wheel_velocity_joint";
  joint_state.position[1] = m_brWheelVelocityJoint->Position(0);
  joint_state.name[2] = "front_left_steer_joint";
  joint_state.position[2] = flWheelAngle;
  joint_state.name[3] = "front_left_wheel_velocity_joint";
  joint_state.position[3] = m_flWheelVelocityJoint->Position(0);
  joint_state.name[4] = "front_right_steer_joint";
  joint_state.position[4] = frWheelAngle;
  joint_state.name[5] = "front_right_wheel_velocity_joint";
  joint_state.position[5] = m_frWheelVelocityJoint->Position(0);

  m_jointPub.publish(joint_state);

  // Publish ground truth pose of the base link in gazebo to TF topic
  geometry_msgs::TransformStamped ground_truth_trans;
  ground_truth_trans.header.frame_id = "ground_truth";
  ground_truth_trans.child_frame_id = "base_link";
  ground_truth_trans.header.stamp = ros::Time::now();
  ground_truth_trans.transform.translation.x =
      worldPos.X() - m_groundTruthOrigin.X();
  ground_truth_trans.transform.translation.y =
      worldPos.Y() - m_groundTruthOrigin.Y();
  ground_truth_trans.transform.translation.z =
      worldPos.Z() - m_groundTruthOrigin.Z();
  ground_truth_trans.transform.rotation.x = worldRot.X();
  ground_truth_trans.transform.rotation.y = worldRot.Y();
  ground_truth_trans.transform.rotation.z = worldRot.Z();
  ground_truth_trans.transform.rotation.w = worldRot.W();
  m_tfBroadcaster.sendTransform(ground_truth_trans);

  // Publish current steering geometry of vehicle
  ackermann_msgs::AckermannSteering currentSteering;
  currentSteering.front_left_steering_angle = flWheelAngle;
  currentSteering.front_right_steering_angle = frWheelAngle;
  currentSteering.speed = currentLinearSpeed;
  m_steeringPub.publish(currentSteering);

  // Update last sim time
  m_lastSimTime = currTime;
}

// Called by the world update start event
void AckermannControlPlugin::controlCallback(
    const geometry_msgs::TwistStamped &cmd) {
  // Lock thread
  std::lock_guard<std::mutex> lock(m_mutex);
  // linear.x represents the desired linear velocity of the vehicle (m/s)
  m_desiredVelocity = cmd.twist.linear.x;
  // angular.z represents the desired steering angle of the vehicle (radians)
  m_desiredSteerAngle = cmd.twist.angular.z;
  ROS_INFO("Desired Velocity: %s m/s",
           std::to_string(m_desiredVelocity).c_str());
  ROS_INFO("Desired Steer Angle: %s radians",
           std::to_string(m_desiredSteerAngle).c_str());

  // See `ackermann_geometry.pdf` for math
  m_desiredLeftWheelAngle = atan2(2 * m_l * sin(m_desiredSteerAngle),
                                  2 * m_l * cos(m_desiredSteerAngle) -
                                      m_w * sin(m_desiredSteerAngle));
  m_desiredRightWheelAngle = atan2(2 * m_l * sin(m_desiredSteerAngle),
                                   2 * m_l * cos(m_desiredSteerAngle) +
                                       m_w * sin(m_desiredSteerAngle));

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