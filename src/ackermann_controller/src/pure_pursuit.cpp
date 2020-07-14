#include <climits>

#include <geometry_msgs/TwistStamped.h>
#include <tf2/utils.h>

#include <ackermann_controller/pure_pursuit.hpp>

PurePursuit::PurePursuit(ros::NodeHandle &privateNH, ros::NodeHandle &publicNH)
    : m_privateNH{privateNH}, m_publicNH{publicNH}, m_tfListener{m_tfBuffer} {
  ROS_INFO("Initialize Pure Pursuit");
  m_privateNH.param<std::string>("vehicle_odom_topic", m_vehicleOdomTopic,
                                 "ground_truth");
  m_privateNH.param<std::string>("vehicle_control_topic", m_vehicleControlTopic,
                                 "cmd_vel");
  m_privateNH.param<std::string>("vehicle_path_topic", m_pathTopic, "path");
  m_privateNH.param("look_ahead_distance", m_lookAheadDistance, 10.0);
  m_privateNH.param("velocity", m_velocity, 5.0);

  m_vehicleSub = m_publicNH.subscribe(m_vehicleOdomTopic, 1,
                                      &PurePursuit::controlCallback, this);
  m_pathSub =
      m_publicNH.subscribe(m_pathTopic, 1, &PurePursuit::pathCallback, this);

  m_controlPub = m_publicNH.advertise<geometry_msgs::TwistStamped>(
      m_vehicleControlTopic, 1);
  m_pathPub = m_publicNH.advertise<nav_msgs::Path>(m_pathTopic, 1);
}

PurePursuit::~PurePursuit() = default;

void PurePursuit::controlCallback(const nav_msgs::Odometry &odom) {

  if (m_path) {

    std::lock_guard<std::mutex> controllerLock(m_controllerMutex);
    m_vehicleState = odom;

    bool deleteFirstPose = PurePursuit::findClosestPointOnPath();
    if (deleteFirstPose) {
      m_path->poses.erase(m_path->poses.begin());
    }
    if (m_path->poses.empty()) {
      m_path = boost::none;
      geometry_msgs::TwistStamped cmd;
      m_controlPub.publish(cmd);
      return;
    }
    m_pathPub.publish(m_path.get());

    double vehicleX{m_vehicleState.pose.pose.position.x};
    double vehicleY{m_vehicleState.pose.pose.position.y};

    geometry_msgs::PoseStamped lookAheadPoseOdom;
    lookAheadPoseOdom = m_path->poses.back();
    double poseDistance{0};

    ROS_INFO("Header Before: %s", lookAheadPoseOdom.header.frame_id.c_str());

    for (auto pose : m_path->poses) {
      poseDistance = std::sqrt(std::pow(pose.pose.position.x - vehicleX, 2.0) +
                               std::pow(pose.pose.position.y - vehicleY, 2.0));
      if (poseDistance > m_lookAheadDistance) {
        lookAheadPoseOdom = pose;
        break;
      }
    }

    ROS_INFO("Header After: %s", lookAheadPoseOdom.header.frame_id.c_str());
    ROS_INFO("Path Size: %s", std::to_string(m_path->poses.size()).c_str());

    geometry_msgs::TransformStamped transform;

    geometry_msgs::PoseStamped lookAheadPoseRobot;
    bool notDone{true};
    while (notDone) {
      try {
        transform = m_tfBuffer.lookupTransform(
            "base_link", "ground_truth", ros::Time ::now(), ros::Duration(0.0));
        tf2::doTransform(lookAheadPoseOdom, lookAheadPoseRobot, transform);
        notDone = false;
      } catch (tf2::TransformException &ex) {
        ROS_INFO("OVER HERE");
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
        continue;
      }
    }

    ROS_INFO("transform X: %s",
             std::to_string(transform.transform.translation.x).c_str());
    ROS_INFO("transform Y: %s",
             std::to_string(transform.transform.translation.y).c_str());

    ROS_INFO("Robot X: %s",
             std::to_string(lookAheadPoseRobot.pose.position.x).c_str());
    ROS_INFO("Robot Y: %s",
             std::to_string(lookAheadPoseRobot.pose.position.y).c_str());
    ROS_INFO("Robot Header: %s", lookAheadPoseRobot.header.frame_id.c_str());

    ROS_INFO("Odom X: %s",
             std::to_string(lookAheadPoseOdom.pose.position.x).c_str());
    ROS_INFO("Odom Y: %s",
             std::to_string(lookAheadPoseOdom.pose.position.y).c_str());
    ROS_INFO("Odom Header: %s", lookAheadPoseOdom.header.frame_id.c_str());

    double steeringAngle{2 * lookAheadPoseRobot.pose.position.x /
                         std::pow(poseDistance, 2.0)};

    geometry_msgs::TwistStamped cmd;
    cmd.twist.linear.x = m_velocity;
    cmd.twist.angular.z = steeringAngle;
    ROS_INFO("Steering Angle: %s", std::to_string(steeringAngle).c_str());

    m_controlPub.publish(cmd);
  }
}

void PurePursuit::pathCallback(const nav_msgs::Path &path) {
  std::lock_guard<std::mutex> controllerLock(m_controllerMutex);
  m_path = path;
}

bool PurePursuit::findClosestPointOnPath() {
  double vehicleX{m_vehicleState.pose.pose.position.x};
  double vehicleY{m_vehicleState.pose.pose.position.y};
  double vehicleTheta{tf2::getYaw(m_vehicleState.pose.pose.orientation)};

  double minDistance = DBL_MAX;
  auto firstPoint = m_path->poses.begin();
  auto secondPoint = m_path->poses.begin();
  ++secondPoint;

  auto fistDistance{
      std::sqrt(std::pow(firstPoint->pose.position.x - vehicleX, 2.0) +
                std::pow(firstPoint->pose.position.y - vehicleY, 2.0))};

  auto secondDistance{
      std::sqrt(std::pow(secondPoint->pose.position.x - vehicleX, 2.0) +
                std::pow(secondPoint->pose.position.y - vehicleY, 2.0))};

  return (fistDistance < secondDistance) ? false : true;
}