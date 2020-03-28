/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

class DriveSignal {
  double leftSignal, rightSignal;
  bool neutral_mode_brake;
public:
  DriveSignal(double left, double right, bool brake);
  DriveSignal(double left, double right);
  double getLeft();
  double getRight();
  bool getNeutralMode();
};

