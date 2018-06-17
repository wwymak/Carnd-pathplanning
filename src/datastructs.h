//
// Created by Wing Yee mak on 16/06/2018.
//

#ifndef PATH_PLANNING_DATASTRUCTS_H
#define PATH_PLANNING_DATASTRUCTS_H

#include "Eigen-3.3/Eigen/Eigen"
#include "Eigen-3.3/Eigen/Core"
#include "vector"

struct SensorData
{
	int id;
	double x;
	double y;
	double vx;
	double vy;
	double s;
	double d;
};

struct LocalisationData
{
    int id;
    double x;
    double y;
    double s;
    double d;
    double yaw;
    double speed;
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
    Eigen::VectorXd startState;
    Eigen::VectorXd endState;
    Eigen::VectorXd coeffs_s;
    Eigen::VectorXd coeffs_d;

    vector<double> timesteps;
    vector<double> d;
};

#endif //PATH_PLANNING_DATASTRUCTS_H