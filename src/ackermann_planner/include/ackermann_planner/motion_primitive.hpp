#include <vector>

/// @brief This struct defines a single displacement which when applied to a
/// vehicle state is a motion primitive. This only considers spacial
/// displacements (x, y, theta) and ingores velocity
struct Primitive {
  const double m_deltaX;
  const double m_deltaY;
  const double m_deltaTheta;
  Primitive(const double deltaX, const double deltaY, const double deltaTheta)
      : m_deltaX{deltaX}, m_deltaY{deltaY}, m_deltaTheta{deltaTheta} {}
};

class MotionPrimitive {
public:
  MotionPrimitive(const double wheelBase, const double velocity,
                  const double maxSteerAngle, const double dt,
                  const double angleDiscretization,
                  const int numberOfPrimitives);

  ~MotionPrimitive();

  /// @brief This function calculates the expected changes to the vehicles state
  /// given the steering angle and the other paramters set at object creation
  /// @param steerAngle the steer angle of the vehicle
  void calculateMotionPrimitive(const double steerAngle);

  std::vector<Primitive> getMotionPrimitives();

private:
  int m_numberOfPrimitives;
  double m_angleDiscretization;
  double m_wheelBase;
  double m_arcLength;
  std::vector<Primitive> m_primitiveVector;
};