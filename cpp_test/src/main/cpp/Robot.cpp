/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

void Robot::RobotInit() {
    std::vector<Subsystem*> subsystems{&drivetrain, &arm};
    manager = std::make_unique<SubsystemManager>(subsystems);
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
    manager -> stopDisabledLoop();
    
    manager -> startEnabledLoop();
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
    manager -> stopDisabledLoop();
    
    manager -> startEnabledLoop();
}

void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {
    manager -> stopEnabledLoop();

    manager -> startDisabledLoop();
}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {
    manager -> stopDisabledLoop();
    
    manager -> startEnabledLoop();
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
