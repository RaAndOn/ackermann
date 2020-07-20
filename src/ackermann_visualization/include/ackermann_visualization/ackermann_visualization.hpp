#pragma once

#include <mutex>
#include <ros/ros.h>

#include <visualization_msgs/MarkerArray.h>

#include <ackermann_msgs/AckermannPath.h>

class AckermannVisualization {
public:
  AckermannVisualization(ros::NodeHandle &privateNH, ros::NodeHandle &publicNH);

  ~AckermannVisualization();

private:
  /// @brief Private ROS Node Handle
  ros::NodeHandle m_privateNH;
  /// @brief Public ROS Node Handle
  ros::NodeHandle m_publicNH;

  /// @brief Publisher of path marker visualization
  ros::Publisher m_markerVisualizationPub;

  /// @brief Subscriber to the vehicle state topic
  ros::Subscriber m_pathSub;

  /// @brief Name of the topic which publishes the markers for visualization
  const std::string m_markerVisualizationTopic;

  /// @brief Name of the topic which publishes the vehicle path
  const std::string m_pathTopic;

  /// @brief This function adds a marker to the marker array for publication and
  /// visualization in RVIZ
  /// @param markerArray reference to the marker array
  /// @param pose pose and gear of the new marker being added to the marker
  /// array
  void addMarkerToArray(visualization_msgs::MarkerArray &markerArray,
                        const ackermann_msgs::AckermannPoseStamped &pose);

  void visualizeCallback(const ackermann_msgs::AckermannPath &path);
};
