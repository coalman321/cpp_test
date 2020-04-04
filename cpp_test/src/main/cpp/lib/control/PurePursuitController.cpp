/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "lib/control/PurePursuitController.h"
#include "cmath"
#include "lib/geometry/Twist2d.h"

PurePursuitController::PurePursuitController(double fixed_lookahead, double max_accel, double nominal_dt, Path path, bool reversed, double path_completion_tolerance) : mPath(path) {
    mFixedLookahead = fixed_lookahead;
    mMaxAccel = max_accel;
    mDt = nominal_dt;
    mReversed = reversed;
    mPathCompletionTolerance = path_completion_tolerance;
    mLastCommand = Twist2d();
}

Twist2d PurePursuitController::update(Pose2d robot_pose, double now){
    Pose2d pose = robot_pose;
    if (mReversed) {
        pose = Pose2d(robot_pose.Translation(),
            robot_pose.Rotation() + Rotation2d(M_PI));
    }

    double distance_from_path = mPath.getDistanceFromPath(robot_pose.Translation());
    if (this -> isDone()) {
        return Twist2d(0.0, 0.0, 0.0);
    }

    Waypoint lookahead_point = mPath.getLookaheadWaypoint(robot_pose.Translation(),distance_from_path + mFixedLookahead);

    double speed = lookahead_point.speed;
        if (mReversed) {
            speed *= -1;
        }
    // Ensure we don't accelerate too fast from the previous command
    double dt = now - mLastTime;
    if (mLastCommand == Twist2d(0, 0, 0) && mLastTime == 0) {
        dt = mDt;
    }
    double accel = (speed - mLastCommand.dx) / dt;
    if (accel < -mMaxAccel) {
        speed = mLastCommand.dx - mMaxAccel * dt;
    } else if (accel > mMaxAccel) {
        speed = mLastCommand.dx + mMaxAccel * dt;
    }

    return Twist2d(0.0, 0.0, 0.0);

}

bool PurePursuitController::isDone(){
    double remainingLength = mPath.getRemainingLength();
    return remainingLength <= mPathCompletionTolerance;
}
