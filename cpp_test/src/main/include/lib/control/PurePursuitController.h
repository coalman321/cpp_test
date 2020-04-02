/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <lib/control/Path.h>
#include <lib/geometry/Twist2d.h>

class PurePursuitController {
    static const double kEpsilon;

    double mFixedLookahead;
    Path mPath;
    Twist2d mLastCommand;
    double mLastTime;
    double mMaxAccel;
    double mDt;
    bool mReversed;
    double mPathCompletionTolerance;

 public:
  PurePursuitController(double fixed_lookahead, double max_accel, double nominal_dt, Path path,
            bool reversed, double path_completion_tolerance);

  bool isDone();
  Twist2d update();
  
};

const double PurePursuitController::kEpsilon = 1E-9;
