#include <cmath>
#include <vector>

class MotionPrimitive {
public:
  MotionPrimitive(const double wheelBase, const double velocity,
                  const double maxSteerAngle, const double dt)
      : m_wheelBase{wheelBase} {
    m_distance = velocity * dt;
  }

  ~MotionPrimitive();

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

    arcLength(m_distance) = arcMeasure(theta) * circleRadius(r)
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
  void calculateMotionPrimitive(const double steerAngle) {
    // Radius of the circle made by the rear wheel
    double turningRadius{m_wheelBase / std::tan(steerAngle)};
    double arcMeasure{m_distance / turningRadius};
    double deltaX = m_distance * std::cos(arcMeasure);
    double deltaY = m_distance * std::sin(arcMeasure);
    double deltaTheta = arcMeasure;

    m_deltaX.push_back(deltaX);
    m_deltaY.push_back(deltaY);
    m_deltaTheta.push_back(deltaTheta);
  }

private:
  int m_numberOfPrimitives;
  double m_angleDiscritization;
  double m_wheelBase;
  double m_distance;
  std::vector<double> m_deltaX;
  std::vector<double> m_deltaY;
  std::vector<double> m_deltaTheta;
}