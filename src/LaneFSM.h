//
// decision maker for calculating the appropiate trajectories for lane changing/keeping
//

#ifndef PATH_PLANNING_LANEFSM_H
#define PATH_PLANNING_LANEFSM_H


#include "datastructs.h"
#include "Eigen-3.3/Eigen/Eigen"
#include "Eigen-3.3/Eigen/Core"
#include "OtherVehicles.h"

using namespace Eigen;

class LaneFSM {
public:
    FrenetPath OptimalPath(VectorXd currentState, double currentTime, OtherVehicles otherVehicles);
};


#endif //PATH_PLANNING_LANEFSM_H
