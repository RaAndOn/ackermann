#include <ackermann_project/visualization_utils.hpp>
#include <ackermann_visualization/ackermann_visualization.hpp>

AckermannVisualization::AckermannVisualization(ros::NodeHandle &privateNH,
                                               ros::NodeHandle &publicNH)
    : m_privateNH{privateNH}, m_publicNH{publicNH},
      m_pathTopic{getROSParamString(m_privateNH, "vehicle_path_topic", "path")},
      m_markerVisualizationTopic{getROSParamString(
          m_privateNH, "path_visualization_topic", "path_markers")} {

  // Set publishers and subscribers and services
  m_markerVisualizationPub =
      m_publicNH.advertise<visualization_msgs::MarkerArray>(
          m_markerVisualizationTopic, 1);
  m_pathSub = m_publicNH.subscribe(
      m_pathTopic, 1, &AckermannVisualization::visualizeCallback, this);
}

AckermannVisualization::~AckermannVisualization() = default;

void AckermannVisualization::visualizeCallback(
    const ackermann_msgs::AckermannPath &path) {
  visualization_msgs::MarkerArray markerArray;
  for (auto &pose : path.poses) {
    addMarkerToArray(markerArray, pose);
  }
  m_markerVisualizationPub.publish(markerArray);
}

void AckermannVisualization::addMarkerToArray(
    visualization_msgs::MarkerArray &markerArray,
    const ackermann_msgs::AckermannPoseStamped &pose) {
  // Initialize marker
  visualization_msgs::Marker marker;
  // Select marker color
  std_msgs::ColorRGBA color;
  color.a = 1.0; // Don't forget to set the alpha!
  color.r = 0.0;
  color.g = 1.0;
  color.b = 0.0;
  // Create marker to represent pose and gear
  ackermannMarker(marker, markerArray.markers.size(), pose.pose.position,
                  pose.pose.orientation, color, (Gear)pose.pose.gear);
  // Set Marker specifics
  marker.header.frame_id = pose.header.frame_id;
  marker.header.stamp = ros::Time();

  // Add marker to marker array
  markerArray.markers.emplace_back(marker);
}
