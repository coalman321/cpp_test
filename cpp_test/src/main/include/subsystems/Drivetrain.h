/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <lib/util/DriveSignal.h>
#include <cmath>
#include <algorithm>
#include <subsystems/Subsystem.h>
#include <cstdio>
#include <ctre/Phoenix.h>
#include <Constants.h>
#include <lib/geometry/Rotation2D.h>
#include <lib/control/SynchronousPIDF.h>

enum DriveControlMode{OPEN_LOOP, PATH_FOLLOWING, HEADING_CONTROL, POSE_STABILIZE};

class Drivetrain : public Subsystem{
  DriveSignal arcadeDrive(double xVel, double rVel);

  std::unique_ptr<TalonSRX> frontLeft, frontRight, rearLeft, rearRight;
  std::unique_ptr<PigeonIMU> imu;

  double leftTicks, rightTicks; 
  Rotation2d heading, gyroOffset;

  std::unique_ptr<SynchronousPIDF> headingController;
  
  DriveControlMode controlMode;
  double leftDemand = 0, rightDemand = 0;

 public:
  Drivetrain();
  void readPeriodicInputs() override;
  void onLoop() override;
  void writePeriodicOutputs() override;
  void reset() override;
  void setOpenLoop(DriveSignal signal);
  void setVelocityControl(DriveSignal signal);
  units::degree_t getHeading();
  void setHeading(Rotation2d desiredHeading);
};
