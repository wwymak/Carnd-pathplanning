#include "EgoVehicle.h"
#include "vector"

using namespace std;

EgoVehicle::EgoVehicle(Waypoints mWaypointsMap)
        :mWaypointsMap(mWaypointsMap),
         mCurrentTime(0),
         mCurrentState(VectorXd::Zero(6))
{

};

void EgoVehicle::UpdatePath(CarPositonData pos , vector<double> previous_path_x, vector<double> previous_path_y) {
    m_next_path_x.clear();
    m_next_path_y.clear();

    //todo stuff

    //todo calculate the optimal path from the FSM planner
    FrenetPath fpath = fsm.OptimalPath(mCurrentState, mCurrentTime, otherVehicles);

    //push the updates to the next paths
    int prevpath_size = previous_path_x.size();
    for (int i = 0; i < prevpath_size; i++) {
        m_next_path_x.push_back(previous_path_x[i]);
        m_next_path_y.push_back(previous_path_y[i]);
    }

    //todo add on extra points to next path x, next path y from fsm
}