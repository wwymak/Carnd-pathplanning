//
// Created by Wing Yee mak on 16/06/2018.
//

#ifndef PATH_PLANNING_DATASTRUCTS_H
#define PATH_PLANNING_DATASTRUCTS_H

#include "Eigen-3.3/Eigen/Eigen"
#include "Eigen-3.3/Eigen/Core"
#include "vector"

using namespace std;

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

#endif //PATH_PLANNING_DATASTRUCTS_H