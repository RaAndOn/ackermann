#pragma once

#include <ackermann_project/ackermann_utils.hpp>

enum Gear { FORWARD, REVERSE };

struct State {
  double m_x;
  double m_y;
  double m_theta;
  Gear m_gear;
  State(const double x, const double y, const double theta, const Gear &gear)
      : m_x{x}, m_y{y}, m_theta{ackermann::wrapToPi(theta)}, m_gear{gear} {}
};