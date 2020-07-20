#include <ackermann_project/visualization_utils.hpp>

visualization_msgs::Marker
ackermannMarker(const int id, const geometry_msgs::Point &position,
                const geometry_msgs::Quaternion &orientation,
                const std_msgs::ColorRGBA &color, const Gear &gear) {
  visualization_msgs::Marker marker;
  marker.id = id;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position = position;
  marker.pose.orientation = orientation;
  marker.color = color;
  marker.color.a = 1.0; // Don't forget to set the alpha!

  if (gear == Gear::FORWARD) {
    marker.type = visualization_msgs::Marker::ARROW;
    marker.scale.x = .4;
    marker.scale.y = 0.075;
    marker.scale.z = 0.075;
  } else if (gear == Gear::REVERSE) {
    marker.type = visualization_msgs::Marker::ARROW;
    marker.scale.x = .3;
    marker.scale.y = 0.15;
    marker.scale.z = 0.15;
  } else if (gear == Gear::STOP) {
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
  } else {
    ROS_ERROR("Member variable 'gear' of marker was not recognized");
  }

  return marker;
}