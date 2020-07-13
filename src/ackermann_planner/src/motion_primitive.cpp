#include <cmath>

#include <ros/ros.h>

#include <ackermann_planner/motion_primitive.hpp>

MotionPrimitive::MotionPrimitive(const double wheelBase, const double velocity,
                                 const double dt,
                                 const double angleDiscretizationDegrees,
                                 const int numberOfDiscretizations)
    : m_wheelBase{wheelBase}, m_angleDiscretization{M_PI *
                                                    angleDiscretizationDegrees /
                                                    180},
      m_arcLength{velocity * dt} {
  // Assume vehicle moves the same speed and distance regardless of steer angle
  ROS_INFO("Velocity: %s", std::to_string(velocity).c_str());
  ROS_INFO("dt: %s", std::to_string(dt).c_str());

  calculateMotionPrimitive(0, true);
  calculateMotionPrimitive(0, false);
  for (int i = 1; i <= numberOfDiscretizations; ++i) {
    double steerAngle{m_angleDiscretization * i};
    calculateMotionPrimitive(steerAngle, true);
    if (i == 1) {
      /// TODO: Justify these values
      m_angularResolution = m_primitiveVector.back().m_deltaTheta;
      m_distanceResolution = std::max(m_primitiveVector.back().m_deltaX,
                                      m_primitiveVector.back().m_deltaY);
    }
    calculateMotionPrimitive(-steerAngle, true);
    calculateMotionPrimitive(steerAngle, false);
    calculateMotionPrimitive(-steerAngle, false);
  }
}

MotionPrimitive::~MotionPrimitive() = default;

/*
  The diagrams below explain the math used to calculate the turning radius.
  Also look at the `turning_radius.pdf` for more detailed descriptions.

  Below is a diagram of the turning geometry of a bicycle model.
  The smaller inner circle is formed by the fixed back wheel.
  The larger outer circle is formed by the steerable front wheel.

               |----W----|
         , - ~ ~ ~ - ,          /
     , '               ' ,    /a
   ,           _ _       \\ /----
  ,        , -|_ _|- , /R \\
 ,       '     r|    /'     ,
 ,      '       |a /   '    ,
 ,      '              '    ,
  ,      ' , _ _ _ , '     ,
   ,                      ,
     ,                  '
       ' - , _ _ _ ,  '

  W - wheelBase
  r - radius of the rear wheel turning circle
  R - radius of the front wheel turning circle
  a - steering angle of the front wheel

  This code uses the distance traveled by the rear wheel for the motion
 primatives:

  r = W / tan(a)

  arcLength = arcMeasure(theta) * circleRadius(r)
  arcMeasure(theta) = arcLength / r

Geometry for motion primitive displacements
        , - ~ ~ ~ - ,
    , '           /|  ' ,
  ,              / |      ,
 ,              /  | dy    ,
,              /_t_|________,
,                    dx     ,
,                           ,
 ,                         ,
  ,                       ,
    ,                  , '
      ' - , _ _ _ ,  '

  cos(t) = dx / r
  dx = r * cos(t)

  sin(t) = dy / r
  dy = r * sin(t)

  dt = t
*/
void MotionPrimitive::calculateMotionPrimitive(const double steerAngle,
                                               const bool forward) {
  // Modify the primitive based on the gear of the vehicle
  int gear = forward ? 1 : -1;
  // Radius of the circle made by the rear wheel
  double turningRadius{m_wheelBase / std::tan(steerAngle)};
  double arcMeasure{m_arcLength / turningRadius};
  double deltaX = gear * m_arcLength * std::cos(arcMeasure);
  double deltaY = m_arcLength * std::sin(arcMeasure);
  double deltaTheta = gear * arcMeasure;

  m_primitiveVector.push_back(Primitive{deltaX, deltaY, deltaTheta});
}

std::vector<Primitive> MotionPrimitive::getMotionPrimitives() const {
  return m_primitiveVector;
}