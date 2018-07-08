//
// Created by Wing Yee mak on 24/06/2018.
//

#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H


#include "datastructs.h"
#include "waypoints.h"
#include <iostream>
#include <numeric>
#include "algorithm"
#include "Eigen-3.3/Eigen/Eigen"
#include "Eigen-3.3/Eigen/Core"
#include "util.h"
#include "spline.h"


using namespace std;
using namespace Eigen;


class Trajectory {
public:
    Trajectory();

    Trajectory(const VectorXd& startStateX6, const VectorXd& endStateX6, double durationS, double durationD, double timeStart);

    VectorXd QuinicPolynomialCoeffs(VectorXd startState, VectorXd endState, double T);

    VectorXd QuarticPolynomialCoeffs(VectorXd startState, VectorXd endState, double T);

    FrenetTraj TrajectoryS_Highspeed(VectorXd startState, VectorXd endState, double targetSpeed, double currentTime, double durationS, double durationD);
    vector<FrenetPath> GetFrenetPaths(double current_s,double current_s_dot, double current_s_dotdot,  double current_d,
                                                  double current_d_dot, double current_d_ddotdot,double  target_d);

    VectorXd CalcPositionAt(VectorXd polycoeffs, double time);
    vector<FrenetPath> GetSingleFrenetPaths(double current_s,double current_s_dot, double current_s_dotdot,  double current_d,
                                                 double current_d_dot, double current_d_ddotdot,double  target_d);

    TrajectoryStandard CalcSplineTraj(TrajectoryStandard &previous_xy, CarPositonData &car, Waypoints &wps, double targetSpeed, int targetLane);


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
    // Parameter
    const double MAX_SPEED = mileph2meterps(49.5);  // maximum speed [m/s]
    const double MAX_ACCEL = 10.0 ; // maximum acceleration [m/ss]
    const double MAX_JERK = 10.0 ; // maximum acceleration [m/sss]
    const double MAX_CURVATURE = 1.0;;   // maximum curvature [1/m]
    const double MAX_ROAD_D = 8.0;   //maximum road width [m]
    const double MIN_ROAD_D = 2.0;   //maximum road width [m]
    const double D_ROAD_W = 1.0;   // road width sampling length [m]
    const double DT = 0.02;   // time tick [s]
    const double MAXT = 2.0;   // max prediction time [s]
    const double MINT = 0.8;  // min prediction time [s]
    const double TARGET_SPEED = mileph2meterps(49);   // [m/s]
    const double MIN_TARGET_SPEED = mileph2meterps(40);   // [m/s]
    const double D_T_S = mileph2meterps(0.05);   //target speed sampling length [m/s]
    const double N_S_SAMPLE = 1 ;  // sampling number of target speed
    const double SAFE_DISTANCE_S = MAX_SPEED * 2; //aka the 2 second rule
    const double SAFE_DISTANCE_D = 2;
    const double LANE_SEPARATION = 4;
    const double KJ = 0.1;
    const double KT = 0.1;
    const double KD = 1.0;
    const double KS = 1.0;
    const double KLAT = 1.0;
    const double KLON = 1.0;

};


#endif //PATH_PLANNING_TRAJECTORY_H
