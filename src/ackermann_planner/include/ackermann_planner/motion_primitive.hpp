#pragma once

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
                  const double dt, const double angleDiscretizationDegrees,
                  const int numberOfPrimitives);

  ~MotionPrimitive();

  /// @brief This function calculates the expected changes to the vehicles state
  /// given the steering angle and the other paramters set at object creation
  /// @param steerAngle the steer angle of the vehicle
  /// @param forward indicates whether the vehicle is moving forward or reverse
  void calculateMotionPrimitive(const double steerAngle, const bool forward);

  std::vector<Primitive> getMotionPrimitives() const;

  double getDistanceResolution() const { return m_distanceResolution; }

  double getAngularResolution() const { return m_angularResolution; }

private:
  double m_angleDiscretization;
  double m_angularResolution;
  double m_distanceResolution;
  double m_wheelBase;
  double m_arcLength;
  std::vector<Primitive> m_primitiveVector;
};