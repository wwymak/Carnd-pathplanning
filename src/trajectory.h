//
// Created by Wing Yee mak on 15/06/2018.
//

#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H

#include "Eigen-3.3/Eigen/Eigen"
#include "Eigen-3.3/Eigen/Core"
#include "datastructs.h"
#include "vector"
#include "OtherVehicles.h"

using namespace std;
using namespace Eigen;

class Trajectory {
public:

    Trajectory();
    virtual ~Trajectory();

    double timeInterval = 0.02; //car gets sensor data every 0.02s

    Trajectory(VectorXd startState, VectorXd endState, float duration, float startTime, VectorXd coeffs_s, VectorXd coeffs_d);


    VectorXd QuinicPolynomialCoeffs(VectorXd startState, VectorXd endState, float T);
    //lane keeping trajectory
    VectorXd QuarticPolynomialCoeffs(VectorXd startState, VectorXd endState, float T);

    VectorXd CalcPositionAt(VectorXd polycoeffs, double time);


    FrenetPath OptimalLaneKeepingPath(double s0, double current_d, double current_d_dotdot);

    FrenetPath OptimalLaneChangingPath(double s0, double current_d, double current_d_dotdot);

    vector<FrenetPath> GetFrenetPaths(double current_s,double current_s_dot, double current_s_dotdot,  double current_d,
                                      double current_d_dot, double current_d_ddotdot,double  target_d);

    vector<FrenetPath> GetFrenetPathsSpeedChanging(double current_s,double current_s_dot, double current_s_dotdot,  double current_d,
                                                   double current_d_dot, double current_d_ddotdot);

    vector<FrenetPath> GetValidPaths(vector<FrenetPath> candidatePaths);
    FrenetPath GetOptimalPath(vector<FrenetPath> validPaths);

    bool CheckNoCollision(vector<vector<CarPositonData>> obstacles, FrenetPath path);

private:
    OtherVehicles curr_obstacles;
};


#endif //PATH_PLANNING_TRAJECTORY_H
