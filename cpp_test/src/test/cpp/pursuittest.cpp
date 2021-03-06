#pragma once
#include "lib/control/PurePursuitController.h"
#include "vector"
#include "cstdio"
#include "cmath"
#include "gtest/gtest.h"

TEST(PursuitTracking, SkewRight){
    std::vector<Waypoint> waypoints;
    Waypoint start = Waypoint(Translation2d(0,0), 10);
    Waypoint mid = Waypoint(Translation2d(10,0), 20);
    Waypoint end = Waypoint(Translation2d(10,10), 10);
    waypoints.push_back(start);
    waypoints.push_back(mid);
    waypoints.push_back(end);
    Path path = Path(waypoints);

    double nominalDt = 0.010;
    //slows the convergence down because the pose calculation is not 
    //quite a perfect update from the pose estimator. This model ignores
    //any effects from inertia
    double friction_factor = 1.0; 

    //turn factor induces drag on the controller to simulate a drivetrain
    // that favors turning one direction (right). > 1 will simulate an oversteering
    // while < 1 will simulate a drivetrain that is less reactive
    double turn_factor = 1.0;

    double lookahead = 2.0; //u
    double max_accel = 4; //u/s^2
    bool reversed = false;
    double path_completion_tolerance = 0.01; //u

    PurePursuitController controller = PurePursuitController(lookahead, max_accel, nominalDt, path, reversed, path_completion_tolerance);
    Pose2d robot = Pose2d();

    double t;
    for(t = 0; t < 3.3 && !controller.isDone(); t += nominalDt){
        Twist2d update = controller.update(robot, t);

        //scale the update of the pose by by the friction and turn factors
        update = Twist2d(update.dx * friction_factor * nominalDt, update.dy * 
            friction_factor * nominalDt, update.dtheta * turn_factor * nominalDt);

        //update the pose with the specified transform
        robot = robot.Exp(update);

        /*printf("time: %f robot pose: %s calculated twist linear: %f angular: %f remaining path: %f\n",
            t, robot.toCSV().c_str(), update.dx, update.dtheta, controller.getPathRemaining());*/
    }
    printf("Path following complete! robot stopped with a pose of %s at time: %f\n", robot.toCSV().c_str(), t);

    ASSERT_TRUE(controller.isDone());
}