/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "lib/control/PurePursuitController.h"

PurePursuitController::PurePursuitController(double fixed_lookahead, double max_accel, double nominal_dt, Path path,
            bool reversed, double path_completion_tolerance) {

}

Twist2d PurePursuitController::update(){
    return Twist2d();

}

bool PurePursuitController::isDone(){
    return false;
}
