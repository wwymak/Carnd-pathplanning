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
#include "nanoflann.hpp"
using namespace nanoflann;

#include "KDTreeVectorOfVectorsAdaptor.h"

using namespace std;
using namespace Eigen;

class Waypoints {
public:

    vector<WaypointData> waypointsList;
    const double max_track_s = 6945.554;

    Waypoints();

    vector<double> getFrenet(double x, double y, double theta);
    vector<double> getXY(double s, double d);
    double xySpeedToFrenetSpeed(double Vxy, double s);

//    WaypointData ClosestWaypoint(double x, double y);
//    WaypointData NextWaypoint(double x, double y);

    void calcSplineTrack();

private:

    tk::spline xspline;
    tk::spline yspline;
    tk::spline dx_spline;
    tk::spline dy_spline;

    vector<double> waypoints_x;
    vector<double> waypoints_y;
    vector<double> waypoints_s;
    vector<double> waypoints_dx;
    vector<double> waypoints_dy;
    vector<double> waypoints_normx;
    vector<double> waypoints_normy;

    vector<double> map_s; // pre-computed for faster access

    // better granularity: 1 point per meter
    vector<double> hp_waypoints_x;
    vector<double> hp_waypoints_y;
    vector<double> hp_waypoints_dx;
    vector<double> hp_waypoints_dy;
    vector<double> hp_waypoints_s;
    vector<double> hp_waypoints_d;

    std::vector<double> new_map_s; // pre-computed for faster access

    double max_error = 0.0;
    double sum_error = 0.0;
    double avg_error = 0.0;
    unsigned int num_error = 0;

    double max_s = 6945.554;

//    int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const std::vector<double> &maps_y);
//    int NextWaypoint(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
};


#endif //PATH_PLANNING_WAYPOINTS_H
