//
// Created by Wing Yee mak on 24/06/2018.
//

#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H


#include "datastructs.h"
#include <iostream>
#include <numeric>
#include "algorithm"
#include "Eigen-3.3/Eigen/Eigen"
#include "Eigen-3.3/Eigen/Core"

using namespace std;
using namespace Eigen;


class Trajectory {
public:
//    Trajectory();

    Trajectory(const VectorXd& startStateX6, const VectorXd& endStateX6, double durationS, double durationD, double timeStart);

    VectorXd QuinicPolynomialCoeffs(VectorXd startState, VectorXd endState, float T);

    VectorXd QuarticPolynomialCoeffs(VectorXd startState, VectorXd endState, float T)

private:
    double mDurationS;
    double mDurationD;
    double mTargetSpeed;
    double mTimeStart;
    VectorXd mSCoeffs;
    VectorXd mDCoeffs;
    VectorXd mStartState;
    VectorXd mEndState;
    double mDeltaT = 0.02; //trajectory timesteps (or should this be somehting higher?)
};


#endif //PATH_PLANNING_TRAJECTORY_H
