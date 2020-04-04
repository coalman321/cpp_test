/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#define _USE_MATH_DEFINES

#include "lib/geometry/Rotation2d.h"
#include <cmath>

Rotation2d::Rotation2d()
    : radians(0),
      m_cos(std::cos(0)),
      m_sin(std::sin(0)) {}

Rotation2d::Rotation2d(double angle)
    : radians(angle),
      m_cos(std::cos(radians)),
      m_sin(std::sin(radians)) {}

Rotation2d::Rotation2d(double x, double y) {
  const auto magnitude = std::hypot(x, y);
  if (magnitude > 1e-6) {
    m_sin = y / magnitude;
    m_cos = x / magnitude;
  } else {
    m_sin = 0.0;
    m_cos = 1.0;
  }
  radians = std::atan2(m_sin, m_cos);
}

Rotation2d Rotation2d::operator+(const Rotation2d& other) const {
  return RotateBy(other);
}

Rotation2d& Rotation2d::operator+=(const Rotation2d& other) {
  double cos = Cos() * other.Cos() - Sin() * other.Sin();
  double sin = Cos() * other.Sin() + Sin() * other.Cos();
  m_cos = cos;
  m_sin = sin;
  radians = std::atan2(m_sin, m_cos);
  return *this;
}

Rotation2d Rotation2d::operator-(const Rotation2d& other) const {
  return *this + -other;
}

Rotation2d& Rotation2d::operator-=(const Rotation2d& other) {
  *this += -other;
  return *this;
}

Rotation2d Rotation2d::operator-() const { return Rotation2d(-radians); }

Rotation2d Rotation2d::operator*(double scalar) const {
  return Rotation2d(radians * scalar);
}

bool Rotation2d::operator==(const Rotation2d& other) const {
  return std::abs(radians - other.radians) < 1E-9;
}

bool Rotation2d::operator!=(const Rotation2d& other) const {
  return !operator==(other);
}

Rotation2d Rotation2d::RotateBy(const Rotation2d& other) const {
  return {Cos() * other.Cos() - Sin() * other.Sin(),
          Cos() * other.Sin() + Sin() * other.Cos()};
}