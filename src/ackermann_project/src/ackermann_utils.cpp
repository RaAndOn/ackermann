#include <cmath>

#include <ackermann_project/ackermann_utils.hpp>

namespace ackermann {

double wrapToPi(const double angle) {
  if (angle >= M_PI) {
    return angle - 2 * M_PI;
  } else if (angle < -M_PI) {
    return 2 * M_PI + angle;
  }
  return angle;
}

} // namespace ackermann

std::string getROSParamString(const ros::NodeHandle &nodeHandle,
                              const std::string &paramName,
                              const std::string &defaultValue) {
  std::string paramValue;
  nodeHandle.param<std::string>(paramName, paramValue, defaultValue);
  return paramValue;
}