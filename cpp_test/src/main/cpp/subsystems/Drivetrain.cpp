/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Drivetrain.h"

Drivetrain::Drivetrain() {}

void Drivetrain::readPeriodicInputs(){
    
}

void Drivetrain::onLoop(){
    std::printf("hello from drive!\n");
}

void Drivetrain::writePeriodicOutputs(){
    
}

DriveSignal Drivetrain::arcadeDrive(double xVel, double rVel){
    double maxInput = std::max(std::max(std::abs(xVel - rVel), std::abs(xVel + rVel)), 1.0);
    double rightMotorOutput = (xVel + rVel) / maxInput;
    double leftMotorOutput = (xVel - rVel) / maxInput;

    return DriveSignal(rightMotorOutput, leftMotorOutput);
}
