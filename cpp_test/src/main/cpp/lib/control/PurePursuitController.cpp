/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "lib/control/PurePursuitController.h"
#include "cmath"
#include "lib/util/Util.h"

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

    Waypoint lookahead_point = getLookaheadWaypoint(robot_pose.Translation(), mFixedLookahead);
    Circle circle = joinPath(pose, lookahead_point.position);

    if (isDone()) {
        return Twist2d(0.0, 0.0, 0.0);
    }

    //printf("lookahead point X: %f Y: %f Path joining Circle radius: %f\n",
    //    lookahead_point.position.X(), lookahead_point.position.Y(), circle.radius);

    double speed = lookahead_point.speed;
    if (mReversed) {
        speed *= -1;
    }

    // Ensure we don't accelerate too fast from the previous command
    double dt = now - mLastTime;
    if ( mLastTime == NAN) {
        dt = mDt;
    }
        double accel = (speed - mLastCommand.dx) / dt;
    if (accel < -mMaxAccel) {
        speed = mLastCommand.dx - mMaxAccel * dt;
    } else if (accel > mMaxAccel) {
        speed = mLastCommand.dx + mMaxAccel * dt;
    }

    // Ensure we slow down in time to stop
    // vf^2 = v^2 + 2*a*d
    // 0 = v^2 + 2*a*d
    double remaining_distance = getPathRemaining();
    double max_allowed_speed = std::sqrt(2 * mMaxAccel * remaining_distance);
    if (std::abs(speed) > max_allowed_speed) {
        speed = max_allowed_speed * Util::sgn<double>(speed);
    }
    static double kMinSpeed = 4.0;
    if (std::abs(speed) < kMinSpeed) {
        // Hack for dealing with problems tracking very low speeds with
        // Talons
        speed = kMinSpeed * Util::sgn(speed);
    }

    Twist2d rv;
    //if the circle has a radius of zero, the points are colinear
    //otherwise there is a curvature (radius will be negative for left turns and positive for right)
    if(std::abs(circle.radius) > kEpsilon){
        rv = Twist2d(speed, 0, std::abs(speed) / circle.radius * -1);
    } else {
        rv = Twist2d(speed, 0, 0);
    }

    mLastTime = now;
    mLastCommand = rv;
    return rv;

}

Circle PurePursuitController::joinPath(Pose2d robot_pose, Translation2d lookahead_point){
    double x1 = robot_pose.Translation().X();
    double y1 = robot_pose.Translation().Y();
    double x2 = lookahead_point.X();
    double y2 = lookahead_point.Y();

    Translation2d pose_to_lookahead =  lookahead_point - robot_pose.Translation();
    double cross_product = pose_to_lookahead.X() * robot_pose.Rotation().Sin()
            - pose_to_lookahead.Y() * robot_pose.Rotation().Cos();
    if (std::abs(cross_product) < kEpsilon) {
        return Circle();
    }

    double dx = x1 - x2;
    double dy = y1 - y2;
    double my = (cross_product > 0 ? -1 : 1) * robot_pose.Rotation().Cos();
    double mx = (cross_product > 0 ? 1 : -1) * robot_pose.Rotation().Sin();

    double cross_term = mx * dx + my * dy;

    if (std::abs(cross_term) < kEpsilon) {
        // Points are colinear
        return Circle();
    }
    Circle rv;
    rv.center = Translation2d((mx * (x1 * x1 - x2 * x2 - dy * dy) + 2 * my * x1 * dy) / (2 * cross_term),
                              (-my * (-y1 * y1 + y2 * y2 + dx * dx) + 2 * mx * y1 * dx) / (2 * cross_term));
    rv.radius = .5 * std::abs((dx * dx + dy * dy) / cross_term) * Util::sgn<double>(cross_product);

    return rv;
}

bool PurePursuitController::isDone(){
    double remainingLength = getPathRemaining();
    return remainingLength != NAN && mLastTime != NAN && remainingLength <= mPathCompletionTolerance;
}

Waypoint PurePursuitController::getLookaheadWaypoint(Translation2d currentPosition, double lookahead){
    double interpolatedLookahead = lookahead / mPath.getPathLength();
    ClosestPointReport report = mPath.getClosestPoint(currentPosition);
    path_remaining = mPath.getPathLength() - mPath.getPathLength() * (report.path_interpolant);
    return mPath.interpolatePath(report.path_interpolant + interpolatedLookahead);
}
