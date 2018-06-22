//
// Created by Wing Yee mak on 19/06/2018.
//

#ifndef PATH_PLANNING_EGOVEHICLE_H
#define PATH_PLANNING_EGOVEHICLE_H

#include "vector"
#include "datastructs.h"
#include "trajectory.h"
#include "waypoints.h"

using namespace std;

class EgoVehicle {
public:
    vector<double> next_path_x;
    vector<double> next_path_y;

    Trajectory trajectory;


    void generateLaneChangePath();
    void generateLaneKeepingPath();
    void generateOptimalPath();

    void updatePath(CarPositonData pos , vector<double> previous_path_x, vector<double> previous_path_y, const vector<double> maps_s, const vector<double> maps_x, const vector<double> maps_y);

    vector<double> getNextPathX() {return next_path_x;};
    vector<double> getNextPathY() { return next_path_y;};
private:
    Waypoints roadmap;
};


#endif //PATH_PLANNING_EGOVEHICLE_H
