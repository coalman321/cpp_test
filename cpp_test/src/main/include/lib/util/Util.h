/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

class Util {
 public:
  Util();

  //linearly interpolate between two values
  static double lerp(double v0, double v1, double t) {
    return (1 - t) * v0 + t * v1;
  }

  template <typename T>  static int sgn(T val) {
    return (T(0) < val) - (val < T(0));
  }

};
