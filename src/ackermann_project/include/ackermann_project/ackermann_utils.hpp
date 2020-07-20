#pragma once

#include <ros/ros.h>

/// @brief Wrap an angle from -Pi to Pi
/// @param angle to be wrapped
/// @return wrapped angle
double wrapToPi(const double angle);

/// @brief Function to return the value of requested ROS string param. Very
/// useful for constructor initialization.
/// @param nodeHandle Node handle to check for param
/// @param paramName Name of the param whose value is being gathered
/// @param defaultValue Default value to return if param is not found
/// @return Value of requested ROS param
std::string getROSParamString(const ros::NodeHandle &nodeHandle,
                              const std::string &paramName,
                              const std::string &defaultValue);

/// @brief Function to return the value of requested ROS number param. Very
/// useful for constructor initialization.
/// @param nodeHandle Node handle to check for param
/// @param paramName Name of the param whose value is being gathered
/// @param defaultValue Default value to return if param is not found
/// @return Value of requested ROS param
template <typename T>
T getROSParam(const ros::NodeHandle &nodeHandle, const std::string &paramName,
              const T defaultValue) {
  T paramValue;
  nodeHandle.param<T>(paramName, paramValue, defaultValue);
  return paramValue;
}
