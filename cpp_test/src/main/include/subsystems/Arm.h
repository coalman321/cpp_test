/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <subsystems/Subsystem.h>

class Arm : public Subsystem{
 public:
  Arm();
  void readPeriodicInputs() override;
  void onLoop() override;
  void writePeriodicOutputs() override;
  void reset() override;
};
