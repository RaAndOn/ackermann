#pragma once

#include <visualization_msgs/MarkerArray.h>

#include <ackermann_project/planner_utils.hpp>

visualization_msgs::Marker
ackermannMarker(const int id, const geometry_msgs::Point &position,
                const geometry_msgs::Quaternion &orientation,
                const std_msgs::ColorRGBA &color, const Gear &gear);