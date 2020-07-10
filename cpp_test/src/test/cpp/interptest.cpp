
#pragma once
#include "lib/control/Path.h"
#include "lib/util/Util.h"
#include "vector"
#include "cstdio"
#include "gtest/gtest.h"

TEST(LinearInterpTest, InterpolateStart){
    double start = 10, end = 20;
    double interpolated = Util::lerp(start, end, 0);
    //printf("linear interpolation from 10 to 20 when iterpolation index is 0: %f\n", interpolated);
    ASSERT_EQ(interpolated, start) << "linear interpolation failed. expected: " << start << " got: " << interpolated;
}
TEST(LinearInterpTest, InterpolateMid){
    double start = 10, end = 20, mid = 15;
    double interpolated = Util::lerp(start, end, 0.5);
    //printf("linear interpolation from 10 to 20 when iterpolation index is 0.5: %f\n", interpolated);
    ASSERT_EQ(interpolated, mid) << "linear interpolation failed. expected: " << mid << " got: " << interpolated;
}
TEST(LinearInterpTest, InterpolateEnd){
    double start = 10, end = 20;
    double interpolated = Util::lerp(start, end, 1);
    //printf("linear interpolation from 10 to 20 when iterpolation index is 1: %f\n", interpolated);
    ASSERT_EQ(interpolated, end) << "linear interpolation failed. expected: " << end << " got: " << interpolated;
}

TEST(WaypointTest, InterpolateLow){
    Waypoint first = Waypoint(Translation2d(0,0), 10);
    Waypoint second = Waypoint(Translation2d(10,0), 20);
    Waypoint interp = first.interpolate(second, 0);
    /*printf("interpolating between X: %f Y: %f SPD: %f and X: %f Y: %f SPD: %f at index 0 got X: %f Y: %f SPD: %f\n",
        first.position.X(), first.position.Y(), first.speed, second.position.X(), second.position.Y(), second.speed,
        interp.position.X(), interp.position.Y(), interp.speed);*/
    ASSERT_EQ(interp, first) << "start boundary interpolated waypoint is not start waypoint";
}

TEST(WaypointTest, InterpolateHigh){
    Waypoint first = Waypoint(Translation2d(0,0), 10);
    Waypoint second = Waypoint(Translation2d(10,0), 20);
    Waypoint interp = first.interpolate(second, 1);
    /*printf("interpolating between X: %f Y: %f SPD: %f and X: %f Y: %f SPD: %f at index 1 got X: %f Y: %f SPD: %f\n",
        first.position.X(), first.position.Y(), first.speed, second.position.X(), second.position.Y(), second.speed,
        interp.position.X(), interp.position.Y(), interp.speed);*/
    ASSERT_EQ(interp, second) << "end boundary interpolated waypoint is not end waypoint";
}

TEST(WaypointTest, InterpolateCenter){
    Waypoint first = Waypoint(Translation2d(0,10), 10);
    Waypoint second = Waypoint(Translation2d(10,0), 20);
    Waypoint interp = first.interpolate(second, 0.5);
    /*printf("interpolating between X: %f Y: %f SPD: %f and X: %f Y: %f SPD: %f at index 0.5 got X: %f Y: %f SPD: %f\n",
        first.position.X(), first.position.Y(), first.speed, second.position.X(), second.position.Y(), second.speed,
        interp.position.X(), interp.position.Y(), interp.speed);*/
    ASSERT_EQ(Waypoint(Translation2d(5,5), 15), interp) << "Center interpolated waypoint is not equal to calculated center";
}

TEST(PathInterpolation, InterpolateStart){
    std::vector<Waypoint> waypoints;
    Waypoint start = Waypoint(Translation2d(0,0), 10);
    Waypoint mid = Waypoint(Translation2d(10,0), 20);
    Waypoint end = Waypoint(Translation2d(10,10), 10);
    waypoints.push_back(start);
    waypoints.push_back(mid);
    waypoints.push_back(end);
    Path pathToInterpolate = Path(waypoints);
    ASSERT_EQ(pathToInterpolate.interpolatePath(0), start) << "start of path interpolation is not equal to start of path";
}

TEST(PathInterpolation, InterpolateEnd){
    std::vector<Waypoint> waypoints;
    Waypoint start = Waypoint(Translation2d(0,0), 10);
    Waypoint mid = Waypoint(Translation2d(10,0), 20);
    Waypoint end = Waypoint(Translation2d(10,10), 10);
    waypoints.push_back(start);
    waypoints.push_back(mid);
    waypoints.push_back(end);
    Path pathToInterpolate = Path(waypoints);
    ASSERT_EQ(pathToInterpolate.interpolatePath(1), end) << "end of path does not interpolate to end of path";
}

TEST(PathInterpolation, InterpolateMid){
    std::vector<Waypoint> waypoints;
    Waypoint start = Waypoint(Translation2d(0,0), 10);
    Waypoint mid = Waypoint(Translation2d(10,0), 20);
    Waypoint end = Waypoint(Translation2d(10,10), 10);
    waypoints.push_back(start);
    waypoints.push_back(mid);
    waypoints.push_back(end);
    Path pathToInterpolate = Path(waypoints);
    Waypoint interpolated = pathToInterpolate.interpolatePath(0.5);
    ASSERT_EQ(interpolated, mid) << "section between waypoints does not interpolate properly. expected X: " 
            << mid.position.X() << " Y: " << mid.position.Y() << " SPD: " << mid.speed << " got X:"
            << interpolated.position.X() << " Y: " << interpolated.position.Y() << " SPD: " << interpolated.speed;
}

TEST(PathInterpolation, InterpolateBetween){
    std::vector<Waypoint> waypoints;
    Waypoint start = Waypoint(Translation2d(0,0), 10);
    Waypoint mid = Waypoint(Translation2d(10,0), 20);
    Waypoint end = Waypoint(Translation2d(10,10), 10);
    waypoints.push_back(start);
    waypoints.push_back(mid);
    waypoints.push_back(end);
    Path pathToInterpolate = Path(waypoints);
    Waypoint interpolatedLower = pathToInterpolate.interpolatePath(0.25);
    Waypoint hypotheticalLower = Waypoint(Translation2d(5,0), 15);
    ASSERT_EQ(interpolatedLower, hypotheticalLower) << "section between waypoints does not interpolate properly. expected X: " 
            << hypotheticalLower.position.X() << " Y: " << hypotheticalLower.position.Y() << " SPD: " << hypotheticalLower.speed << " got X:"
            << interpolatedLower.position.X() << " Y: " << interpolatedLower.position.Y() << " SPD: " << interpolatedLower.speed;


    Waypoint hypotheticalUpper = Waypoint(Translation2d(10,5), 15);
    Waypoint interpolatedUpper = pathToInterpolate.interpolatePath(0.75);
    ASSERT_EQ(interpolatedUpper, hypotheticalUpper) << "section between waypoints does not interpolate properly. expected X: " 
            << hypotheticalUpper.position.X() << " Y: " << hypotheticalUpper.position.Y() << " SPD: " << hypotheticalUpper.speed << " got X:"
            << interpolatedUpper.position.X() << " Y: " << interpolatedUpper.position.Y() << " SPD: " << interpolatedUpper.speed;
}