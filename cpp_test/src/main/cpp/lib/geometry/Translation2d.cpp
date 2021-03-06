/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "lib/geometry/Translation2d.h"
#include "cmath"

Translation2d::Translation2d() {}

Translation2d::Translation2d(double x, double y) : m_x(x), m_y(y) {}

double Translation2d::Distance(const Translation2d& other) const {
  return std::hypot(other.m_x - m_x, other.m_y - m_y);
}

double Translation2d::Norm() const {
  return std::hypot(m_x, m_y);
}

Translation2d Translation2d::RotateBy(const Rotation2d& other) const {
  return {m_x * other.Cos() - m_y * other.Sin(),
          m_x * other.Sin() + m_y * other.Cos()};
}

Translation2d Translation2d::interpolate(const Translation2d& other, double x){
  if (x <= 0) {
    return *this;
  } else if (x >= 1) {
    return Translation2d(other);
  }
  return Translation2d(x * (other.m_x - m_x) + m_x, x * (other.m_y - m_y) + m_y);
}


Translation2d Translation2d::operator+(const Translation2d& other) const {
  return {X() + other.X(), Y() + other.Y()};
}

Translation2d& Translation2d::operator+=(const Translation2d& other) {
  m_x += other.m_x;
  m_y += other.m_y;
  return *this;
}

Translation2d Translation2d::operator-(const Translation2d& other) const {
  return *this + -other;
}

Translation2d& Translation2d::operator-=(const Translation2d& other) {
  *this += -other;
  return *this;
}

Translation2d Translation2d::operator-() const { return {-m_x, -m_y}; }

Translation2d Translation2d::operator*(double scalar) const {
  return {scalar * m_x, scalar * m_y};
}

Translation2d& Translation2d::operator*=(double scalar) {
  m_x *= scalar;
  m_y *= scalar;
  return *this;
}

Translation2d Translation2d::operator/(double scalar) const {
  return *this * (1.0 / scalar);
}

bool Translation2d::operator==(const Translation2d& other) const {
  return std::abs(m_x - other.m_x) < 1E-9 &&
         std::abs(m_y - other.m_y) < 1E-9;
}

bool Translation2d::operator!=(const Translation2d& other) const {
  return !operator==(other);
}

Translation2d& Translation2d::operator/=(double scalar) {
  *this *= (1.0 / scalar);
  return *this;
}
