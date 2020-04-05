/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <lib/control/Path.h>
#include <lib/geometry/Pose2d.h>

struct Circle {
  public:
    Translation2d center;
    //negative radius confers turning left while a positive radius turns right. the points are colinear if radius = 0
    double radius;
};

class PurePursuitController {
    static constexpr double kEpsilon = 1E-9;

    double mFixedLookahead;
    Path mPath;
    Twist2d mLastCommand;
    double mLastTime;
    double mMaxAccel;
    double mDt;
    bool mReversed;
    double mPathCompletionTolerance;
    double path_remaining;

 public:
  PurePursuitController(double fixed_lookahead, double max_accel, double nominal_dt, Path path,
            bool reversed, double path_completion_tolerance);

  //gets the current status of the controller and if it has finished tracking the path
  bool isDone();

  //gets the latest command from the controller
  Twist2d update(Pose2d robot_pose, double now);

  //internal function for joining the current pose of the robot and the desired path point
  Circle joinPath(Pose2d robot_pose, Translation2d lookahead_point);

  Waypoint getLookaheadWaypoint(Translation2d currentPathPoint, double lookahead);

  double getPathRemaining(){
    return path_remaining;
  }
};

