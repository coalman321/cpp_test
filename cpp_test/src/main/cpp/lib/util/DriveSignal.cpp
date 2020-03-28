/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "lib/util/DriveSignal.h"

DriveSignal::DriveSignal(double left, double right, bool brake){
    neutral_mode_brake = brake;
    leftSignal = left;
    rightSignal = right;
}

DriveSignal::DriveSignal(double left, double right){
    neutral_mode_brake = false;
    leftSignal = left;
    rightSignal = right;
}

double DriveSignal::getLeft(){
    return leftSignal;
}

double DriveSignal::getRight(){
    return rightSignal;
}

bool DriveSignal::getNeutralMode(){
    return neutral_mode_brake;
}
