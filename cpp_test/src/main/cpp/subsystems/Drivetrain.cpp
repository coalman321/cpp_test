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
    if(controlMode == OPEN_LOOP){
        frontLeft -> Set(ControlMode::PercentOutput, leftDemand);
        frontRight -> Set(ControlMode::PercentOutput, rightDemand);
    } else {
        frontLeft -> Set(ControlMode::Velocity, leftDemand);
        frontRight -> Set(ControlMode::Velocity, rightDemand);
    }
}

void Drivetrain::reset(){
    controlMode = OPEN_LOOP;
    headingController.reset();
    headingController = std::make_unique<SynchronousPIDF>(DRIVE_HEADING_KP, DRIVE_HEADING_KI, DRIVE_HEADING_KD, DRIVE_HEADING_KF);
    setHeading(Rotation2d());

    //config talons 
    frontLeft -> ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 100);
    frontLeft -> SetStatusFramePeriod(Status_2_Feedback0, 5, 100);
    frontLeft -> SetSensorPhase(true);
    frontLeft -> SelectProfileSlot(0, 0);
    frontLeft -> Config_kP(DRIVE_LEFT_KP, 0);
    frontLeft -> Config_kI(DRIVE_LEFT_KI, 0);
    frontLeft -> Config_kD(DRIVE_LEFT_KD, 0);
    frontLeft -> Config_kF(DRIVE_LEFT_KF, 0);
    frontLeft -> Config_IntegralZone(DRIVE_LEFT_I_MAX, 0);
    frontLeft -> SetInverted(false);
    frontLeft -> SetNeutralMode(NeutralMode::Brake);
    frontLeft -> ConfigVoltageCompSaturation(DRIVE_VCOMP);
    frontLeft -> EnableVoltageCompensation(true);

    rearLeft -> SetInverted(false);
    rearLeft -> SetNeutralMode(NeutralMode::Brake);
    rearLeft -> ConfigVoltageCompSaturation(DRIVE_VCOMP);
    rearLeft -> EnableVoltageCompensation(true);
    rearLeft -> Follow(*frontLeft);

    frontRight -> ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 100);
    frontRight -> SetStatusFramePeriod(Status_2_Feedback0, 5, 100);
    frontRight -> SetSensorPhase(true);
    frontRight -> SelectProfileSlot(0, 0);
    frontRight -> Config_kP(DRIVE_LEFT_KP, 0);
    frontRight -> Config_kI(DRIVE_LEFT_KI, 0);
    frontRight -> Config_kD(DRIVE_LEFT_KD, 0);
    frontRight -> Config_kF(DRIVE_LEFT_KF, 0);
    frontRight -> Config_IntegralZone(DRIVE_LEFT_I_MAX, 0);
    frontRight -> SetInverted(false);
    frontRight -> SetNeutralMode(NeutralMode::Brake);
    frontRight -> ConfigVoltageCompSaturation(DRIVE_VCOMP);
    frontRight -> EnableVoltageCompensation(true);

    rearRight -> SetInverted(false);
    rearRight -> SetNeutralMode(NeutralMode::Brake);
    rearRight -> ConfigVoltageCompSaturation(DRIVE_VCOMP);
    rearRight -> EnableVoltageCompensation(true);
    rearRight -> Follow(*frontRight);

    frontLeft -> SetSelectedSensorPosition(0, 0, 0);
    frontRight -> SetSelectedSensorPosition(0, 0, 0);


}

units::degree_t Drivetrain::getHeading(){
    return heading.Degrees();
}

void Drivetrain::setHeading(Rotation2d desiredHeading){
    printf("SET heading %d", heading.Degrees());
    gyroOffset = desiredHeading.RotateBy(Rotation2d(units::degree_t(imu -> GetFusedHeading())).inverse());
    printf("Gyro offset: %d", gyroOffset.Degrees());
    heading = desiredHeading;
}

DriveSignal Drivetrain::arcadeDrive(double xVel, double rVel){
    double maxInput = std::max(std::max(std::abs(xVel - rVel), std::abs(xVel + rVel)), 1.0);
    double rightMotorOutput = (xVel + rVel) / maxInput;
    double leftMotorOutput = (xVel - rVel) / maxInput;

    return DriveSignal(rightMotorOutput, leftMotorOutput);
}
