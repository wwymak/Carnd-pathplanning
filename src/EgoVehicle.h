//
// Created by Wing Yee mak on 19/06/2018.
//

#ifndef PATH_PLANNING_EGOVEHICLE_H
#define PATH_PLANNING_EGOVEHICLE_H

#include "vector"
#include "datastructs.h"
#include "trajectory.h"
#include "waypoints.h"
#include "LaneFSM.h"
#include "Eigen-3.3/Eigen/Eigen"
#include "Eigen-3.3/Eigen/Core"

using namespace Eigen;
using namespace std;

class EgoVehicle {
public:
    EgoVehicle(Waypoints& mWaypointsMap, LaneFSM& fsm, OtherVehicles& otherVehicles);
    vector<double> m_next_path_x;
    vector<double> m_next_path_y;


    void UpdatePath(CarPositonData pos , vector<double> previous_path_x, vector<double> previous_path_y);

    vector<double> getNextPathX() {return m_next_path_x;};
    vector<double> getNextPathY() { return m_next_path_y;};
private:
    Waypoints& mWaypointsMap;
    LaneFSM& fsm;
    const double mMaxPredictTime = 2; //secs
    VectorXd mCurrentState;
    double mCurrentTime;
    OtherVehicles& otherVehicles;
};


#endif //PATH_PLANNING_EGOVEHICLE_H
