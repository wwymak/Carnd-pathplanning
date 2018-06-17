//
// Created by Wing Yee mak on 15/06/2018.
//

#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H

#include "Eigen-3.3/Eigen/Eigen"
#include "Eigen-3.3/Eigen/Core"
#include "datastructs.h"
#include "vector"

using namespace Eigen;

class Trajectory {
public:
//    state s, s_dot, s_dotdot, d, d_dot, d_dotdot
// jerkD = d_dotdotdot = d_dotdot/duration
//jerkS = s_dotdotdot = s_dotdot/duration

    Trajectory();
    virtual ~Trajectory();

    double timeInterval = 0.02; //car gets sensor data every 0.02s

    Trajectory(VectorXd startState, VectorXd endState, float duration, float startTime, VectorXd coeffs_s, VectorXd coeffs_d);

    double  CalcJerkAt(VectorXd coeffs, double T);
    double CalcAccelAt(VectorXd coeffs, double time)
    double JerkCost(double duration);
    double AccelCost(double duration);
    double SpeedCost(double duration, double targetSpeed);
    double ClosenessCost();
    double LaneCenternessCost();

    VectorXd QuinicPolynomialCoeffs(VectorXd startState, VectorXd endState, float T);
    //lane keeping trajectory
    VectorXd QuarticPolynomialCoeffs(VectorXd startState, VectorXd endState, float T);

    VectorXd CalcStateAt(VectorXd polycoeffs, double time);

    FrenetPath Trajectory::VelocityKeepingPath() {

    }

    FrenetPath Trajectory::GeneralPath(double s0, double current_d, double current_d_dotdot);

    vector<FrenetPath> Trajectory::GetFrenetPaths(double current_speed, double current_d,
                                                  double current_d_dot, double current_d_ddotdot, double s0);

};


#endif //PATH_PLANNING_TRAJECTORY_H
