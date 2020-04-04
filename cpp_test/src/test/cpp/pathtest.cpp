
#pragma once
#include "lib/control/Path.h"
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
    printf("creating path\n");
    ClosestPointReport report = path.getClosestPoint(Translation2d(0, 1));
    printf("closest point is at X: %f Y: %f and a distance of %f with a path interpolant of %f\n", report.closest_point.X(), report.closest_point.Y(), report.distance, report.path_interpolant);
    ASSERT_EQ(report.closest_point, start.position) << "does not find nearest point as start";
    ASSERT_EQ(report.distance, 1.0) << "incorrect distance from nearest point expected 1.0 got: " << report.path_interpolant;
    ASSERT_EQ(report.path_interpolant, 0) << "path interpolant expected to be 0.0 got: " << report.path_interpolant;
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
    printf("creating path\n");
    ClosestPointReport report = path.getClosestPoint(Translation2d(11, -1));
    printf("closest point is at X: %f Y: %f and a distance of %f with a path interpolant of %f\n", report.closest_point.X(), report.closest_point.Y(), report.distance, report.path_interpolant);
    ASSERT_EQ(report.closest_point, mid.position) << "does not find nearest point as mid";
    ASSERT_EQ(report.distance, std::sqrt(2)) << "incorrect distance from nearest point expected 1.0 got: " << report.path_interpolant;
    ASSERT_EQ(report.path_interpolant, 0.5) << "path interpolant expected to be 0.5 got: " << report.path_interpolant;
}