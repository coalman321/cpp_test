/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/TimedRobot.h>
#include <SubsystemManager.h>
#include <SubsystemManager.h>
#include <subsystems/Drivetrain.h>
#include <subsystems/Arm.h>
#include <subsystems/Subsystem.h>


class Robot : public frc::TimedRobot {
  std::unique_ptr<SubsystemManager> manager;
  Drivetrain drivetrain;
  Arm arm;

 public:
  void RobotInit() override;
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;
};
