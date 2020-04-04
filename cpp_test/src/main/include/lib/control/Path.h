/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <lib/geometry/Translation2d.h>
#include <vector>
#include <string>

struct Waypoint {
  Translation2d position;
  double speed;
  std::string name = "";

  public:
    Waypoint(Translation2d pos, double speed){
      position = pos; 
      this -> speed = speed;
    }

    Waypoint(Translation2d wayPosition, double waySpeed, std::string wayName){
      position = wayPosition; 
      speed = waySpeed;
      name = wayName;
    }

    // interpolate between two waypoints bounded by [0 - 1]
    Waypoint interpolate(Waypoint other, double location);

    /**
     * Checks equality between this Waypoint and another object.
     *
     * @param other The other object.
     * @return Whether the two objects are equal.
     */
    bool operator==(const Waypoint &other) const{
      return position == other.position &&
            std::abs(speed - other.speed) < 1E-9;
  }
};

struct ClosestPointReport {
  public:
    double path_interpolant;
    Translation2d closest_point;
    double distance = 0;
};

class Path {

  std::vector<Waypoint> waypoints;
  double pathLength;

 public:
  Path(std::vector<Waypoint> waypoints);
  double getRemainingLength();
  double getDistanceFromPath(Translation2d current);
  Waypoint getLookaheadWaypoint(Translation2d currentPathPoint, double lookahead);

  // interpolate the path from start to end bounded by [0 - 1]
  Waypoint interpolatePath(double location);

  ClosestPointReport getClosestPoint(Translation2d query_point);

};
