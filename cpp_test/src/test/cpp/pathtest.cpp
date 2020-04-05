
#pragma once
#include "lib/control/PurePursuitController.h"
#include "vector"
#include "cstdio"
#include "cmath"
#include "gtest/gtest.h"

TEST(ClosestPointReports, ReportStart){
    std::vector<Waypoint> waypoints;
    Waypoint start = Waypoint(Translation2d(0,0), 10);
    Waypoint mid = Waypoint(Translation2d(10,0), 20);
    Waypoint end = Waypoint(Translation2d(10,10), 10);
    waypoints.push_back(start);
    waypoints.push_back(mid);
    waypoints.push_back(end);
    Path path = Path(waypoints);
    ClosestPointReport report = path.getClosestPoint(Translation2d(0, 1));
    printf("closest point is at X: %f Y: %f and a distance of %f with a path interpolant of %f\n", report.closest_point.X(), report.closest_point.Y(), report.distance, report.path_interpolant);
    ASSERT_EQ(report.closest_point, start.position) << "does not find nearest point as start";
    ASSERT_EQ(report.distance, 1.0) << "incorrect distance from nearest point expected 1.0 got: " << report.path_interpolant;
    ASSERT_EQ(report.path_interpolant, 0) << "path interpolant expected to be 0.0 got: " << report.path_interpolant;
}

TEST(ClosestPointReports, ReportMid){
    std::vector<Waypoint> waypoints;
    Waypoint start = Waypoint(Translation2d(0,0), 10);
    Waypoint mid = Waypoint(Translation2d(10,0), 20);
    Waypoint end = Waypoint(Translation2d(10,10), 10);
    waypoints.push_back(start);
    waypoints.push_back(mid);
    waypoints.push_back(end);
    Path path = Path(waypoints);
    ClosestPointReport report = path.getClosestPoint(Translation2d(11, -1));
    printf("closest point is at X: %f Y: %f and a distance of %f with a path interpolant of %f\n", report.closest_point.X(), report.closest_point.Y(), report.distance, report.path_interpolant);
    ASSERT_EQ(report.closest_point, mid.position) << "does not find nearest point as mid";
    ASSERT_EQ(report.distance, std::sqrt(2)) << "incorrect distance from nearest point expected 1.0 got: " << report.path_interpolant;
    ASSERT_EQ(report.path_interpolant, 0.5) << "path interpolant expected to be 0.5 got: " << report.path_interpolant;
}

TEST(ClosestPointReports, ReportEnd){
    std::vector<Waypoint> waypoints;
    Waypoint start = Waypoint(Translation2d(0,0), 10);
    Waypoint mid = Waypoint(Translation2d(10,0), 20);
    Waypoint end = Waypoint(Translation2d(10,10), 10);
    waypoints.push_back(start);
    waypoints.push_back(mid);
    waypoints.push_back(end);
    Path path = Path(waypoints);
    ClosestPointReport report = path.getClosestPoint(Translation2d(11, 10));
    printf("closest point is at X: %f Y: %f and a distance of %f with a path interpolant of %f\n", report.closest_point.X(), report.closest_point.Y(), report.distance, report.path_interpolant);
    ASSERT_EQ(report.closest_point, end.position) << "does not find nearest point as end";
    ASSERT_EQ(report.distance, 1) << "incorrect distance from nearest point expected 1.0 got: " << report.path_interpolant;
    ASSERT_EQ(report.path_interpolant, 1.0) << "path interpolant expected to be 1.0 got: " << report.path_interpolant;
}

TEST(ClosestPointReports, ReportInterpolatedUpper){
    std::vector<Waypoint> waypoints;
    Waypoint start = Waypoint(Translation2d(0,0), 10);
    Waypoint mid = Waypoint(Translation2d(10,0), 20);
    Waypoint end = Waypoint(Translation2d(10,10), 10);
    waypoints.push_back(start);
    waypoints.push_back(mid);
    waypoints.push_back(end);
    Path path = Path(waypoints);
    ClosestPointReport report = path.getClosestPoint(Translation2d(11, 5));
    printf("closest point is at X: %f Y: %f and a distance of %f with a path interpolant of %f\n", report.closest_point.X(), report.closest_point.Y(), report.distance, report.path_interpolant);
    ASSERT_EQ(report.closest_point, Translation2d(10, 5)) << "does not find nearest point as upper interpolated";
    ASSERT_EQ(report.distance, 1) << "incorrect distance from nearest point expected 1.0 got: " << report.path_interpolant;
    ASSERT_EQ(report.path_interpolant, 0.75) << "path interpolant expected to be 1.0 got: " << report.path_interpolant;
}

TEST(ClosestPointReports, ReportInterpolatedLower){
    std::vector<Waypoint> waypoints;
    Waypoint start = Waypoint(Translation2d(0,0), 10);
    Waypoint mid = Waypoint(Translation2d(10,0), 20);
    Waypoint end = Waypoint(Translation2d(10,10), 10);
    waypoints.push_back(start);
    waypoints.push_back(mid);
    waypoints.push_back(end);
    Path path = Path(waypoints);
    ClosestPointReport report = path.getClosestPoint(Translation2d(5, 1));
    printf("closest point is at X: %f Y: %f and a distance of %f with a path interpolant of %f\n", report.closest_point.X(), report.closest_point.Y(), report.distance, report.path_interpolant);
    ASSERT_EQ(report.closest_point, Translation2d(5, 0)) << "does not find nearest point as lower interpolated";
    ASSERT_EQ(report.distance, 1) << "incorrect distance from nearest point expected 1.0 got: " << report.path_interpolant;
    ASSERT_EQ(report.path_interpolant, 0.25) << "path interpolant expected to be 1.0 got: " << report.path_interpolant;
}

TEST(Lookahead, LookaheadStart){
    std::vector<Waypoint> waypoints;
    Waypoint start = Waypoint(Translation2d(0,0), 10);
    Waypoint mid = Waypoint(Translation2d(10,0), 20);
    Waypoint end = Waypoint(Translation2d(10,10), 10);
    waypoints.push_back(start);
    waypoints.push_back(mid);
    waypoints.push_back(end);
    Path path = Path(waypoints);

    double nominalDt = 0.010; //s
    double lookaheadDist = 2.0; //u
    double max_accel = 4; //u/s^2
    bool reversed = false;
    double path_completion_tolerance = 1.0; //u
    PurePursuitController controller = PurePursuitController(lookaheadDist, max_accel, nominalDt, path, reversed, path_completion_tolerance);

    Waypoint lookahead = controller.getLookaheadWaypoint(Translation2d(-1, 0), 1);
    printf("Lookahead point is at X: %f Y: %f with a SPD: %f\n", lookahead.position.X(), lookahead.position.Y(), lookahead.speed);
    ASSERT_EQ(lookahead, Waypoint(Translation2d(1,0), 11)) << "does not find correct lookahead";
}

TEST(Lookahead, LookaheadEnd){
    std::vector<Waypoint> waypoints;
    Waypoint start = Waypoint(Translation2d(0,0), 10);
    Waypoint mid = Waypoint(Translation2d(10,0), 20);
    Waypoint end = Waypoint(Translation2d(10,10), 10);
    waypoints.push_back(start);
    waypoints.push_back(mid);
    waypoints.push_back(end);
    Path path = Path(waypoints);

    double nominalDt = 0.010; //s
    double lookaheadDist = 2.0; //u
    double max_accel = 4; //u/s^2
    bool reversed = false;
    double path_completion_tolerance = 1.0; //u
    PurePursuitController controller = PurePursuitController(lookaheadDist, max_accel, nominalDt, path, reversed, path_completion_tolerance);

    Waypoint lookahead = controller.getLookaheadWaypoint(Translation2d(10, 9.5), 1);
    printf("Lookahead point is at X: %f Y: %f with a SPD: %f\n", lookahead.position.X(), lookahead.position.Y(), lookahead.speed);
    ASSERT_EQ(lookahead, end) << "does not find correct lookahead";
}

TEST(Lookahead, LookaheadMid){
    std::vector<Waypoint> waypoints;
    Waypoint start = Waypoint(Translation2d(0,0), 10);
    Waypoint mid = Waypoint(Translation2d(10,0), 20);
    Waypoint end = Waypoint(Translation2d(10,10), 10);
    waypoints.push_back(start);
    waypoints.push_back(mid);
    waypoints.push_back(end);
    Path path = Path(waypoints);

    double nominalDt = 0.010; //s
    double lookaheadDist = 2.0; //u
    double max_accel = 4; //u/s^2
    bool reversed = false;
    double path_completion_tolerance = 1.0; //u
    PurePursuitController controller = PurePursuitController(lookaheadDist, max_accel, nominalDt, path, reversed, path_completion_tolerance);

    Waypoint lookahead = controller.getLookaheadWaypoint(Translation2d(10, -1), 1);
    printf("Lookahead point is at X: %f Y: %f with a SPD: %f\n", lookahead.position.X(), lookahead.position.Y(), lookahead.speed);
    ASSERT_EQ(lookahead, Waypoint(Translation2d(10,1), 19)) << "does not find correct lookahead";
}
