//
// Created by Wing Yee mak on 14/06/2018.
//

#ifndef PATH_PLANNING_PLANNER_H
#define PATH_PLANNING_PLANNER_H

#include <iostream>
#include <vector>
#include "Eigen-3.3/unsupported/Eigen/Polynomials"
#include "frenetpath.h"


using namespace std;

class Planner {
public:
    vector<double> waypoints_s;
    vector<double> waypoints_d;
    vector<double>obstacles_s;
    vector<double>obstacles_d;

    bool check_collision(Frenetpath path, vector<double> obstacles_s, vector<double> obstacles_d);
};


#endif //PATH_PLANNING_PLANNER_H
