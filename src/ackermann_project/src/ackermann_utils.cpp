#include <cmath>

#include <ackermann_project/ackermann_utils.hpp>

double wrapToPi(double angle) {
  while (angle >= M_PI) {
    angle -= 2 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2 * M_PI;
  }
  return angle;
}

std::string getROSParamString(const ros::NodeHandle &nodeHandle,
                              const std::string &paramName,
                              const std::string &defaultValue) {
  std::string paramValue;
  nodeHandle.param<std::string>(paramName, paramValue, defaultValue);
  return paramValue;
}
