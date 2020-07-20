#pragma once

#include <ros/ros.h>

/// @brief Wrap an angle from -Pi to Pi
/// @param angle to be wrapped
/// @return wrapped angle
double wrapToPi(const double angle);

std::string getROSParamString(const ros::NodeHandle &nodeHandle,
                              const std::string &paramName,
                              const std::string &defaultValue);

template <typename T>
T getROSParam(const ros::NodeHandle &nodeHandle, const std::string &paramName,
              const T defaultValue) {
  T paramValue;
  nodeHandle.param<T>(paramName, paramValue, defaultValue);
  return paramValue;
}
