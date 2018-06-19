//
// Created by Wing Yee mak on 19/06/2018.
//

#ifndef PATH_PLANNING_EGOVEHICLE_H
#define PATH_PLANNING_EGOVEHICLE_H

#include <vector>
#include "datastructs.h"

using namespace std;

class EgoVehicle {
public:
    vector<double> next_path_x;
    vector<double> next_path_y;
    void updatePath(CarPositonData pos , vector<double> previous_path_x, vector<double> previous_path_y);

    vector<double> getNextPathX() {return next_path_x;};
    vector<double> getNextPathY() { return next_path_y;};
};


#endif //PATH_PLANNING_EGOVEHICLE_H
