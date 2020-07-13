#pragma once

namespace ackermann {

/// @brief Wrap an angle from -Pi to Pi
/// @param angle to be wrapped
/// @return wrapped angle
double wrapToPi(const double angle);

} // namespace ackermann