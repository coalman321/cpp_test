/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Drivetrain.h"

Drivetrain::Drivetrain() {
    frontLeft = std::make_unique<TalonSRX>(DRIVE_FRONT_LEFT_ID);
    frontRight = std::make_unique<TalonSRX>(DRIVE_FRONT_RIGHT_ID);
    rearLeft = std::make_unique<TalonSRX>(DRIVE_REAR_LEFT_ID);
    rearRight = std::make_unique<TalonSRX>(DRIVE_REAR_RIGHT_ID);
    imu = std::make_unique<PigeonIMU>(DRIVE_PIGEON_ID);
    reset();
}

void Drivetrain::readPeriodicInputs(){
    heading = Rotation2d(units::degree_t(imu -> GetFusedHeading()));
}

void Drivetrain::onLoop(){
    
}

void Drivetrain::writePeriodicOutputs(){
    
}

void Drivetrain::reset(){
    controlMode = OPEN_LOOP;
    heading = Rotation2d();
}

units::degree_t Drivetrain::getHeading(){
    return heading.Degrees();
}


DriveSignal Drivetrain::arcadeDrive(double xVel, double rVel){
    double maxInput = std::max(std::max(std::abs(xVel - rVel), std::abs(xVel + rVel)), 1.0);
    double rightMotorOutput = (xVel + rVel) / maxInput;
    double leftMotorOutput = (xVel - rVel) / maxInput;

    return DriveSignal(rightMotorOutput, leftMotorOutput);
}
