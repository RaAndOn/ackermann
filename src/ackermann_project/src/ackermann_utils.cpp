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