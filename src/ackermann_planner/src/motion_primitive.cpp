#include <cmath>

#include <ros/ros.h>

#include <ackermann_planner/motion_primitive.hpp>

MotionPrimitive::MotionPrimitive(const double wheelBase, const double velocity,
                                 const double maxSteerAngle, const double dt,
                                 const double angleDiscretization,
                                 const int numberOfPrimitives)
    : m_wheelBase{wheelBase}, m_angleDiscretization{angleDiscretization},
      m_numberOfPrimitives{numberOfPrimitives} {
  // Assume vehicle moves the same speed and distance regardless of steer angle
  m_arcLength = velocity * dt;
  //
  if (numberOfPrimitives % 2 == 0) {
    ROS_INFO("numberOfPrimitives must be odd, adding one to provided number");
    m_numberOfPrimitives = m_numberOfPrimitives + 1;
  }

  calculateMotionPrimitive(0);
  calculateMotionPrimitive(M_PI / 16);
  calculateMotionPrimitive(-M_PI / 16);
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
void MotionPrimitive::calculateMotionPrimitive(const double steerAngle) {
  /// TODO: Figure out reversing
  // Radius of the circle made by the rear wheel
  double turningRadius{m_wheelBase / std::tan(steerAngle)};
  double arcMeasure{m_arcLength / turningRadius};
  double deltaX = m_arcLength * std::cos(arcMeasure);
  double deltaY = m_arcLength * std::sin(arcMeasure);
  double deltaTheta = arcMeasure;

  m_primitiveVector.push_back(Primitive{deltaX, deltaY, deltaTheta});
}

std::vector<Primitive> MotionPrimitive::getMotionPrimitives() {
  return m_primitiveVector;
}