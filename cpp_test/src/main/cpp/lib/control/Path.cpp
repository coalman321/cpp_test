/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "lib/control/Path.h"
#include "cmath"
#include "cstdio"
#include "algorithm"
#include "lib/util/Util.h"

Waypoint Waypoint::interpolate(Waypoint other, double location){
    Translation2d interpolated = position.interpolate(other.position, location);
    double interpolated_speed = Util::lerp(speed, other.speed, location); 

    return Waypoint(interpolated, interpolated_speed); 
}

Path::Path(std::vector<Waypoint> waypoints) {
    this->waypoints = waypoints;

    //calculate the length of the path
    pathLength = 0;
    if(waypoints.size() > 2){
        for(int i = 0; i < waypoints.size() - 1; i++){
            pathLength += waypoints.at(i).position.Distance(waypoints.at(i + 1).position);
        }
    }
}

double Path::getRemainingLength(){
    return 0.0;
}

double Path::getDistanceFromPath(Translation2d current){
    return 0.0;
}

ClosestPointReport Path::getClosestPoint(Translation2d query_point){
    if(waypoints.size() < 2){
        printf("not enough points in the path to locate a closest point\n");
        return ClosestPointReport();
    } 

    //printf("locating nearest points\n");
    //locate the two closest segments
    int nearestDefined1, nearestDefined2;
    nearestDefined1 = 0, nearestDefined2 = 1;
    for(int i = 0; i < waypoints.size(); i++){
        if(waypoints.at(i).position.Distance(query_point) < waypoints.at(nearestDefined1).position.Distance(query_point)){
            nearestDefined1 = i;
        }
    }
    nearestDefined2 = nearestDefined1 + 1;

    //handle case at the end where we might try to extrapolate
    if(nearestDefined1 == waypoints.size() - 1) {
        nearestDefined2 = nearestDefined1;
        nearestDefined1--;
    }

    //printf("found nearest defined waypoints with list index of %d and %d\n",nearestDefined1, nearestDefined2);

    //calculate distance to nearest point via dot product
    Translation2d nearestStartToEnd = waypoints.at(nearestDefined2).position - waypoints.at(nearestDefined1).position;
    Translation2d nearestStartToQuery = query_point - waypoints.at(nearestDefined1).position;

    //printf("Start to end X: %f Y: %f  Start to query X: %f Y: %f\n", nearestStartToEnd.X(), nearestStartToEnd.Y(),
    //     nearestStartToQuery.X(), nearestStartToQuery.X());

    //get the closest point via the projection
    double dotProd = nearestStartToEnd.X() * nearestStartToQuery.X() + nearestStartToEnd.Y() * nearestStartToQuery.Y();
    double proj = std::clamp(dotProd / nearestStartToEnd.Norm() / nearestStartToEnd.Norm(), 0.0, 1.0);

    //printf("Dot prod: %f Projection: %f\n", dotProd, proj);

    ClosestPointReport report;
    report.closest_point = waypoints.at(nearestDefined1).position.interpolate(waypoints.at(nearestDefined2).position, proj);
    report.path_interpolant = (nearestDefined1 + proj) / ((double) waypoints.size() - 1);
    report.distance = interpolatePath(report.path_interpolant).position.Distance(query_point); 
    return report;

}

Waypoint Path::interpolatePath(double location){
    //bound to [0 , 1]
    if(location <= 0) return waypoints.at(0);
    if(location >= 1) return waypoints.at(waypoints.size() - 1);

    //get the interpolated index of the "lower" point
    int vector_index = int(location * (waypoints.size() - 1));
    double interpolate_index = location * (waypoints.size() - 1) - vector_index;

    //handle case where the index lands on an existing waypoint
    if(interpolate_index < 1E-9) return waypoints.at(vector_index);

    //find the interpolated values
    return waypoints.at(vector_index).interpolate(waypoints.at(vector_index + 1), interpolate_index);
}

Waypoint Path::getLookaheadWaypoint(Translation2d currentPosition, double lookahead){
    double interpolatedLookahead = lookahead / pathLength;
    ClosestPointReport report = getClosestPoint(currentPosition);
    return interpolatePath(report.path_interpolant + interpolatedLookahead);
}
