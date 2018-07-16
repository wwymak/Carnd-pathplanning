//
// Created by Wing Yee mak on 16/06/2018.
//

#ifndef PATH_PLANNING_DATASTRUCTS_H
#define PATH_PLANNING_DATASTRUCTS_H

#include "Eigen-3.3/Eigen/Eigen"
#include "Eigen-3.3/Eigen/Core"
#include "vector"

using namespace std;
using namespace Eigen;

struct SensorFusionData
{
	int id;
	double x;
	double y;
	double vx;
	double vy;
	double s;
	double d;
};

struct CarPositonData
{
    int id;
    double x;
    double y;
    double s;
    double d;
    double yaw;
    double speed;
    int lane;
    int targetLane;
    double targetSpeed;

};

struct WaypointData
{
    double x;
    double y;
    double s;
    double d_x;
    double d_y;
};

struct FrenetPath
{

    vector<double> timesteps;
    vector<double> d;
    vector<double> d_dot;
    vector<double> d_dotdot;
    vector<double> d_dotdotdot;
    vector<double> s;
    vector<double> s_dot;
    vector<double> s_dotdot;
    vector<double> s_dotdotdot;

    double cost_d;
    double cost_s;
    double cost_total;

};

struct FrenetTraj
{
	VectorXd startState;
	VectorXd endState;
	VectorXd sCoeffs;
	VectorXd dCoeffs;
	double durationS;
	double durationD;
};

struct TrajectoryStandard {
	vector<double> x_pts;
	vector<double> y_pts;
	double path_end_s;
	double path_end_d;
	TrajectoryStandard (vector<double> X={}, vector<double> Y={}) : x_pts(X), y_pts(Y) {}
};

#endif //PATH_PLANNING_DATASTRUCTS_H