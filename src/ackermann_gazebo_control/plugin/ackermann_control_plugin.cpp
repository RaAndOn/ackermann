#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>

#include "ackermann_control_plugin.hpp"

namespace gazebo {
class AckermannControlPluginPrivate {
  /// \enum DirectionType
  /// \brief Direction selector switch type.
public:
  enum DirectionType {
    /// \brief Reverse
    REVERSE = -1,
    /// \brief Neutral
    NEUTRAL = 0,
    /// \brief Forward
    FORWARD = 1
  };

public:
  ros::NodeHandle nh;

public:
  ros::Subscriber controlSub;

  /// \brief Pointer to the world
public:
  physics::WorldPtr world;

  /// \brief Pointer to the parent model
public:
  physics::ModelPtr model;

  /// \brief Transport node
public:
  transport::NodePtr gznode;

  /// \brief Ignition transport node
public:
  ignition::transport::Node node;

  /// \brief Ignition transport position pub
public:
  ignition::transport::Node::Publisher posePub;

  /// \brief Physics update event connection
public:
  event::ConnectionPtr updateConnection;

  /// \brief Chassis link
public:
  physics::LinkPtr chassisLink;

  /// \brief Front left wheel joint
public:
  physics::JointPtr flWheelJoint;

  //   /// \brief Front right wheel joint
  // public:
  //   physics::JointPtr frWheelJoint;

  //   /// \brief Rear left wheel joint
  // public:
  //   physics::JointPtr blWheelJoint;

  //   /// \brief Rear right wheel joint
  // public:
  //   physics::JointPtr brWheelJoint;

  /// \brief Front left wheel steering joint
public:
  physics::JointPtr flWheelSteeringJoint;

  //   /// \brief Front right wheel steering joint
  // public:
  //   physics::JointPtr frWheelSteeringJoint;

  /// \brief PID control for the front left wheel steering joint
public:
  common::PID flWheelSteeringPID;

  //   /// \brief PID control for the front right wheel steering joint
  // public:
  //   common::PID frWheelSteeringPID;

  //   /// \brief Last pose msg time
  // public:
  //   common::Time lastMsgTime;

  //   /// \brief Last sim time received
  // public:
  //   common::Time lastSimTime;

  //   /// \brief Last sim time when a steering command is received
  // public:
  //   common::Time lastSteeringCmdTime;

  //   /// \brief Current direction of the vehicle: FORWARD, NEUTRAL, REVERSE.
  // public:
  //   DirectionType directionState;

  //   /// \brief Max torque that can be applied to the front wheels
  // public:
  //   double frontTorque = 0;

  //   /// \brief Max torque that can be applied to the back wheels
  // public:
  //   double backTorque = 0;

  //   /// \brief Max speed (m/s) of the car
  // public:
  //   double maxSpeed = 0;

  //   /// \brief Max steering angle
  // public:
  //   double maxSteer = 0;

  /// \brief Front left wheel desired steering angle (radians)
public:
  double flWheelSteeringCmd = 0;

  //   /// \brief Front right wheel desired steering angle (radians)
  // public:
  //   double frWheelSteeringCmd = 0;

  //   /// \brief Front left wheel radius
  // public:
  //   double flWheelRadius = 0;

  //   /// \brief Front right wheel radius
  // public:
  //   double frWheelRadius = 0;

  //   /// \brief Rear left wheel radius
  // public:
  //   double blWheelRadius = 0;

  //   /// \brief Rear right wheel radius
  // public:
  //   double brWheelRadius = 0;

  //   /// \brief Distance distance between front and rear axles
  // public:
  //   double wheelbaseLength = 0;

  //   /// \brief Distance distance between front left and right wheels
  // public:
  //   double frontTrackWidth = 0;

  //   /// \brief Distance distance between rear left and right wheels
  // public:
  //   double backTrackWidth = 0;

  /// \brief Steering angle of front left wheel at last update (radians)
public:
  double flSteeringAngle = 0;

  //   /// \brief Steering angle of front right wheel at last update (radians)
  // public:
  //   double frSteeringAngle = 0;

  /// \brief Linear velocity of chassis c.g. in world frame at last update (m/s)
public:
  ignition::math::Vector3d chassisLinearVelocity;

  /// \brief Angular velocity of front left wheel at last update (rad/s)
public:
  double flWheelAngularVelocity = 0;

  //   /// \brief Angular velocity of front right wheel at last update (rad/s)
  // public:
  //   double frWheelAngularVelocity = 0;

  //   /// \brief Angular velocity of back left wheel at last update (rad/s)
  // public:
  //   double blWheelAngularVelocity = 0;

  //   /// \brief Angular velocity of back right wheel at last update (rad/s)
  // public:
  //   double brWheelAngularVelocity = 0;

  /// \brief Mutex to protect updates
public:
  std::mutex mutex;

  //   /// \brief Odometer
  // public:
  //   double odom = 0.0;
};
} // namespace gazebo

using namespace gazebo;

/////////////////////////////////////////////////
AckermannControlPlugin::~AckermannControlPlugin() {
  this->m_dataPtr->updateConnection.reset();
}

/////////////////////////////////////////////////
void AckermannControlPlugin::Load(physics::ModelPtr model,
                                  sdf::ElementPtr sdf) {
  gzwarn << "AckermannControlPlugin loading params" << std::endl;
  // shortcut to this->m_dataPtr
  AckermannControlPluginPrivate *dPtr = this->m_dataPtr.get();

  this->m_dataPtr->model = model;
  this->m_dataPtr->world = this->m_dataPtr->model->GetWorld();
  auto physicsEngine = this->m_dataPtr->world->Physics();
  physicsEngine->SetParam("friction_model", std::string("cone_model"));

  this->m_dataPtr->gznode = transport::NodePtr(new transport::Node());
  this->m_dataPtr->gznode->Init();

  if (sdf->HasElement("robotNamespace"))
    this->m_robotNamespace =
        sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
  ros::NodeHandle nh(this->m_robotNamespace);

  // this->m_dataPtr->nh.Subscribe(
  //     "/cmd_vel", &AckermannControlPlugin::VelocityCommandCallback, this);

  // this->m_dataPtr->node.Subscribe(
  //     "/cmd_vel", &AckermannControlPlugin::VelocityCommandCallback, this);

  this->m_dataPtr->posePub =
      this->m_dataPtr->node.Advertise<ignition::msgs::Pose>("/ackermann/pose");

  std::string chassisLinkName =
      dPtr->model->GetName() + "::" + sdf->Get<std::string>("chassis");
  dPtr->chassisLink = dPtr->model->GetLink(chassisLinkName);
  if (!dPtr->chassisLink) {
    std::cerr << "could not find chassis link" << std::endl;
    return;
  }

  std::string flWheelJointName = this->m_dataPtr->model->GetName() + "::" +
                                 sdf->Get<std::string>("front_left_wheel");
  this->m_dataPtr->flWheelJoint =
      this->m_dataPtr->model->GetJoint(flWheelJointName);
  if (!this->m_dataPtr->flWheelJoint) {
    std::cerr << "could not find front left wheel joint" << std::endl;
    return;
  }

  //   std::string frWheelJointName = this->m_dataPtr->model->GetName() + "::" +
  //                                  sdf->Get<std::string>("front_right_wheel");
  //   this->m_dataPtr->frWheelJoint =
  //       this->m_dataPtr->model->GetJoint(frWheelJointName);
  //   if (!this->m_dataPtr->frWheelJoint) {
  //     std::cerr << "could not find front right wheel joint" << std::endl;
  //     return;
  //   }

  //   std::string blWheelJointName = this->m_dataPtr->model->GetName() + "::" +
  //                                  sdf->Get<std::string>("back_left_wheel");
  //   this->m_dataPtr->blWheelJoint =
  //       this->m_dataPtr->model->GetJoint(blWheelJointName);
  //   if (!this->m_dataPtr->blWheelJoint) {
  //     std::cerr << "could not find back left wheel joint" << std::endl;
  //     return;
  //   }

  //   std::string brWheelJointName = this->m_dataPtr->model->GetName() + "::" +
  //                                  sdf->Get<std::string>("back_right_wheel");
  //   this->m_dataPtr->brWheelJoint =
  //       this->m_dataPtr->model->GetJoint(brWheelJointName);
  //   if (!this->m_dataPtr->brWheelJoint) {
  //     std::cerr << "could not find back right wheel joint" << std::endl;
  //     return;
  //   }

  std::string flWheelSteeringJointName =
      this->m_dataPtr->model->GetName() +
      "::" + sdf->Get<std::string>("front_left_wheel_steering");
  this->m_dataPtr->flWheelSteeringJoint =
      this->m_dataPtr->model->GetJoint(flWheelSteeringJointName);
  if (!this->m_dataPtr->flWheelSteeringJoint) {
    std::cerr << "could not find front left steering joint" << std::endl;
    return;
  }

  //   std::string frWheelSteeringJointName =
  //       this->m_dataPtr->model->GetName() +
  //       "::" + sdf->Get<std::string>("front_right_wheel_steering");
  //   this->m_dataPtr->frWheelSteeringJoint =
  //       this->m_dataPtr->model->GetJoint(frWheelSteeringJointName);
  //   if (!this->m_dataPtr->frWheelSteeringJoint) {
  //     std::cerr << "could not find front right steering joint" << std::endl;
  //     return;
  //   }

  std::string paramName;
  double paramDefault;

  // paramName = "front_torque";
  // paramDefault = 0;
  // if (sdf->HasElement(paramName))
  //   this->m_dataPtr->frontTorque = sdf->Get<double>(paramName);
  // else
  //   this->m_dataPtr->frontTorque = paramDefault;

  // paramName = "back_torque";
  // paramDefault = 2000;
  // if (sdf->HasElement(paramName))
  //   this->m_dataPtr->backTorque = sdf->Get<double>(paramName);
  // else
  //   this->m_dataPtr->backTorque = paramDefault;

  // paramName = "max_speed";
  // paramDefault = 10;
  // if (sdf->HasElement(paramName))
  //   this->m_dataPtr->maxSpeed = sdf->Get<double>(paramName);
  // else
  //   this->m_dataPtr->maxSpeed = paramDefault;

  // paramName = "max_steer";
  // paramDefault = 0.6;
  // if (sdf->HasElement(paramName))
  //   this->m_dataPtr->maxSteer = sdf->Get<double>(paramName);
  // else
  //   this->m_dataPtr->maxSteer = paramDefault;

  paramName = "flwheel_steering_p_gain";
  paramDefault = 0;
  if (sdf->HasElement(paramName))
    this->m_dataPtr->flWheelSteeringPID.SetPGain(sdf->Get<double>(paramName));
  else
    this->m_dataPtr->flWheelSteeringPID.SetPGain(paramDefault);

  // paramName = "frwheel_steering_p_gain";
  // paramDefault = 0;
  // if (sdf->HasElement(paramName))
  //   this->m_dataPtr->frWheelSteeringPID.SetPGain(sdf->Get<double>(paramName));
  // else
  //   this->m_dataPtr->frWheelSteeringPID.SetPGain(paramDefault);

  paramName = "flwheel_steering_i_gain";
  paramDefault = 0;
  if (sdf->HasElement(paramName))
    this->m_dataPtr->flWheelSteeringPID.SetIGain(sdf->Get<double>(paramName));
  else
    this->m_dataPtr->flWheelSteeringPID.SetIGain(paramDefault);

  // paramName = "frwheel_steering_i_gain";
  // paramDefault = 0;
  // if (sdf->HasElement(paramName))
  //   this->m_dataPtr->frWheelSteeringPID.SetIGain(sdf->Get<double>(paramName));
  // else
  //   this->m_dataPtr->frWheelSteeringPID.SetIGain(paramDefault);

  paramName = "flwheel_steering_d_gain";
  paramDefault = 0;
  if (sdf->HasElement(paramName))
    this->m_dataPtr->flWheelSteeringPID.SetDGain(sdf->Get<double>(paramName));
  else
    this->m_dataPtr->flWheelSteeringPID.SetDGain(paramDefault);

  // paramName = "frwheel_steering_d_gain";
  // paramDefault = 0;
  // if (sdf->HasElement(paramName))
  //   this->m_dataPtr->frWheelSteeringPID.SetDGain(sdf->Get<double>(paramName));
  // else
  //   this->m_dataPtr->frWheelSteeringPID.SetDGain(paramDefault);

  // // Update wheel radius for each wheel from SDF collision objects
  // //  assumes that wheel link is child of joint (and not parent of joint)
  // //  assumes that wheel link has only one collision
  // unsigned int id = 0;
  // this->m_dataPtr->flWheelRadius = this->CollisionRadius(
  //     this->m_dataPtr->flWheelJoint->GetChild()->GetCollision(id));
  // this->m_dataPtr->frWheelRadius = this->CollisionRadius(
  //     this->m_dataPtr->frWheelJoint->GetChild()->GetCollision(id));
  // this->m_dataPtr->blWheelRadius = this->CollisionRadius(
  //     this->m_dataPtr->blWheelJoint->GetChild()->GetCollision(id));
  // this->m_dataPtr->brWheelRadius = this->CollisionRadius(
  //     this->m_dataPtr->brWheelJoint->GetChild()->GetCollision(id));

  //   // Compute wheelbase, frontTrackWidth, and rearTrackWidth
  //   //  first compute the positions of the 4 wheel centers
  //   //  again assumes wheel link is child of joint and has only one collision
  //   ignition::math::Vector3d flCenterPos =
  //   this->m_dataPtr->flWheelJoint->GetChild()
  //                                              ->GetCollision(id)
  //                                              ->WorldPose()
  //                                              .Pos();
  //   ignition::math::Vector3d frCenterPos =
  //   this->m_dataPtr->frWheelJoint->GetChild()
  //                                              ->GetCollision(id)
  //                                              ->WorldPose()
  //                                              .Pos();
  //   ignition::math::Vector3d blCenterPos =
  //   this->m_dataPtr->blWheelJoint->GetChild()
  //                                              ->GetCollision(id)
  //                                              ->WorldPose()
  //                                              .Pos();
  //   ignition::math::Vector3d brCenterPos =
  //   this->m_dataPtr->brWheelJoint->GetChild()
  //                                              ->GetCollision(id)
  //                                              ->WorldPose()
  //                                              .Pos();

  //   // track widths are computed first
  //   ignition::math::Vector3d vec3 = flCenterPos - frCenterPos;
  //   this->m_dataPtr->frontTrackWidth = vec3.Length();
  //   vec3 = flCenterPos - frCenterPos;
  //   this->m_dataPtr->backTrackWidth = vec3.Length();
  //   // to compute wheelbase, first position of axle centers are computed
  //   ignition::math::Vector3d frontAxlePos = (flCenterPos + frCenterPos) / 2;
  //   ignition::math::Vector3d backAxlePos = (blCenterPos + brCenterPos) / 2;
  //   // then the wheelbase is the distance between the axle centers
  //   vec3 = frontAxlePos - backAxlePos;
  //   this->m_dataPtr->wheelbaseLength = vec3.Length();

  //   //   Max force that can be applied to wheel steering joints
  //   double kMaxSteeringForceMagnitude = 5000;

  //   this->m_dataPtr->flWheelSteeringPID.SetCmdMax(kMaxSteeringForceMagnitude);
  //   this->m_dataPtr->flWheelSteeringPID.SetCmdMin(-kMaxSteeringForceMagnitude);

  //   this->m_dataPtr->frWheelSteeringPID.SetCmdMax(kMaxSteeringForceMagnitude);
  //   this->m_dataPtr->frWheelSteeringPID.SetCmdMin(-kMaxSteeringForceMagnitude);

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->m_dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&AckermannControlPlugin::Update, this));
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(AckermannControlPlugin)
