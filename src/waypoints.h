//
// Created by Wing Yee mak on 18/06/2018.
//

#ifndef PATH_PLANNING_WAYPOINTS_H
#define PATH_PLANNING_WAYPOINTS_H

#include "spline.h"
#include "datastructs.h"
#include "util.h"
#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;
using namespace Eigen;

class Waypoints {
public:

    vector<WaypointData> waypointsList;

    Waypoints();

    void GetSplineTrack();

    vector<double> getFrenet(double x, double y, double theta);

// Transform from Frenet s,d coordinates to Cartesian x,y
    vector<double> getXY(double s, double d);
    WaypointData ClosestWaypoint(double x, double y);
    WaypointData NextWaypoint(double x, double y);

    void calcSplineTrack();

private:
    // The max s value before wrapping around the track back to 0
    const double max_track_s = 6945.554;
    tk::spline xspline;
    tk::spline yspline;
};


#endif //PATH_PLANNING_WAYPOINTS_H
