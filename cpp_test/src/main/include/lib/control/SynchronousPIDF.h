/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

class SynchronousPIDF {
  double kp = 0, ki = 0, kd = 0, kf = 0, iMax = 0, timeDelta = 0.010;
 public:
  SynchronousPIDF(double p, double i, double d, double f){
    kp = p, ki = i, kd = d, kf = f;
  }

  SynchronousPIDF(double p, double i, double d, double f, double iLim, double dt){
    kp = p, ki = i, kd = d, kf = f, iMax = iLim, timeDelta = dt;
  }
  
  void setContinuous();
  void 
};
