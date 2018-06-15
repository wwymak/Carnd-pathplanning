//
// Created by Wing Yee mak on 15/06/2018.
//

#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H

#include "Eigen-3.3/Eigen/Eigen"
#include "Eigen-3.3/Eigen/Core"

using namespace Eigen;

class Trajectory {
public:
//    state s, s_dot, s_dotdot, d, d_dot, d_dotdot
// jerkD = d_dotdotdot = d_dotdot/duration
//jerkS = s_dotdotdot = s_dotdot/duration

    Trajectory();
    virtual ~Trajectory();

    void Init(VectorXd startState, VectorXd endState, float duration, float startTime);

    double JerkCost(double duration, double maxJerkS, double maxJerkD);
    double AccelCost(double duration);
    double SpeedCost(double duration, double targetSpeed);
    double ClosenessCost();
    double LaneCenternessCost();

    VectorXd QuinicPolynomialCoeffs(VectorXd startState, VectorXd endState, float T);
    //lane keeping trajectory
    VectorXd QuarticPolynomialCoeffs(VectorXd startState, VectorXd endState, float T);

};


#endif //PATH_PLANNING_TRAJECTORY_H
