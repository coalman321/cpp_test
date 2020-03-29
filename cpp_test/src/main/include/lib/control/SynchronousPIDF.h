/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <mutex>
#include <vector>
#include <cmath>
#include <algorithm>

/**
 * A synchronous, thread safe, PIDF controller implementation
 */
class SynchronousPIDF {
 protected:
  double kp = 0, ki = 0, kd = 0, kf = 0, iMax = 0, timeDelta = 0.010;
  double error = 0, lastError = 0, derivative = 0, integral = 0, inputRange = 0, setPoint = 0;
  bool continuous = false, enabled = false;
  std::mutex calculationMutex;

  double getContinuousError(double error);
  double clamp(double input, double lower, double upper);

 public:
  /**
   *constructs a new pidf with the specified gains 
   */
  SynchronousPIDF(double p, double i, double d, double f){
    kp = p, ki = i, kd = d, kf = f;
  }

  SynchronousPIDF(double p, double i, double d, double f, double iLim, double dt){
    kp = p, ki = i, kd = d, kf = f, iMax = iLim, timeDelta = dt;
  }
  
  void reset();
  void setContinuous(bool continous);
  void setInputRange(double lower, double upper);
  void updateSetPoint(double set);
  void setPIDF(double kP, double kI, double kD, double kF);
  void setILim(double limit){iMax = limit;};

  double update(double input);

  std::vector<double> getPIF();
  double getError();
};
