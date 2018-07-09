//
// decision maker for calculating the appropiate trajectories for lane changing/keeping
//

#ifndef PATH_PLANNING_LANEFSM_H
#define PATH_PLANNING_LANEFSM_H


#include "datastructs.h"
#include "Eigen-3.3/Eigen/Eigen"
#include "Eigen-3.3/Eigen/Core"
#include "OtherVehicles.h"
#include "RoadState.h"
#include "trajectory.h"
#include "waypoints.h"

using namespace Eigen;

class LaneFSM {
public:

    vector<double> next_x_path;
    vector<double> next_y_path;

    LaneFSM(RoadState& rs, Waypoints& wps);
    FrenetPath OptimalPath(VectorXd currentState, double currentTime, OtherVehicles otherVehicles);

    int CanChangeLane();

    bool ShouldPrepareChange();

    bool laneChangingState = false;

    void LaneKeeping(CarPositonData& egoVehicle, double predictionTimeWindow);

//    void GenerateLaneKeeping();
private:
    RoadState& rs;
    Waypoints& wps;
};


#endif //PATH_PLANNING_LANEFSM_H
