/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <lib/util/DriveSignal.h>

class Drivetrain {
  DriveSignal arcadeDrive(double xVel, double rVel);
 public:
  Drivetrain();
  void setOpenLoop(DriveSignal signal);
  void setVelocityControl(DriveSignal signal);
  
};
