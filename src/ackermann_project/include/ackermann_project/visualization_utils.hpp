#pragma once

#include <visualization_msgs/MarkerArray.h>

#include <ackermann_project/planner_utils.hpp>

/// @brief Function to create marker to represent the pose and gear of a vehicle
/// @param marker reference to marker to modify
/// @param id Marker id
/// @param position Position of marker
/// @param orientation Orientation of marker
/// @param color Color of marker
/// @param gear Gear of vehicle to specify which marker to use
void ackermannMarker(visualization_msgs::Marker &marker, const int id,
                     const geometry_msgs::Point &position,
                     const geometry_msgs::Quaternion &orientation,
                     const std_msgs::ColorRGBA &color, const Gear &gear);