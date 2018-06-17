#include "util.h"
#include "trajectory.h"
#include "datastructs.h"
#include <iostream>

#include "Eigen-3.3/Eigen/Eigen"
#include "Eigen-3.3/Eigen/Core"

//using Eigen::MatrixXd;
//using Eigen::VectorXd;
using namespace std;
using namespace Eigen;

// Parameter
const double MAX_SPEED = mileph2meterps(49.5);  // maximum speed [m/s]
const double MAX_ACCEL = 10.0 ; // maximum acceleration [m/ss]
const double MAX_JERK = 10.0 ; // maximum acceleration [m/sss]
const double MAX_CURVATURE = 1.0;;   // maximum curvature [1/m]
const double MAX_ROAD_WIDTH = 7.0;   //maximum road width [m]
const double D_ROAD_W = 1.0;   // road width sampling length [m]
const double DT = 0.02;   // time tick [s]
const double MAXT = 5.0;   // max prediction time [s]
const double MINT = 2.0;  // min prediction time [s]
const double TARGET_SPEED = mileph2meterps(49);   // [m/s]
const double D_T_S = 5.0 / 3.6;   //target speed sampling length [m/s]
const double N_S_SAMPLE = 1 ;  // sampling number of target speed


// cost weights
const double KJ = 0.1;
const double KT = 0.1;
const double KD = 1.0;
const double KLAT = 1.0;
const double KLON = 1.0;


Trajectory::Trajectory() {}

Trajectory::~Trajectory() {}

VectorXd Trajectory::QuinicPolynomialCoeffs(VectorXd startState, VectorXd endState, float T){

    const auto s0 = startState(0);
    const auto s0_dot = startState(1);
    const auto s0_dotdot = startState(2);

    const auto s1 = endState(0);
    const auto s1_dot = endState(1);
    const auto s1_dotdot = endState(2);



    const auto a0 = s0;
    const auto a1 = s0_dot;
    const auto a2 = 0.5 * s0_dotdot;

    const auto T2 = T * T;
    const auto T3 = T2 * T;
    const auto T4 = T3 * T;
    const auto T5 = T4 * T;

    MatrixXd A(3, 3);
    A << T5, T4, T3,
        5 * T4, 4 * T3, 3 * T2,
        20 * T3, 12 * T2, 6 * T;

    VectorXd b(3);
    b << s1 - a0 - a1 * T - a2 * T2, s1_dot - a1 - 2 * a2 * T , s1_dotdot - a2;

    VectorXd C = A.colPivHouseholderQr().solve(b);
    const auto a5 = C(0);
    const auto a4 = C(1);
    const auto a3 = C(2);

    VectorXd polycoeffs(6);
    polycoeffs << a0, a1, a2,a3, a4, a5;

    return polycoeffs;


}

VectorXd Trajectory::QuarticPolynomialCoeffs(VectorXd startState, VectorXd endState, float T){

    const auto s0 = startState(0);
    const auto s0_dot = startState(1);
    const auto s0_dotdot = startState(2);

    const auto s1 = endState(0);
    const auto s1_dot = endState(1);
    const auto s1_dotdot = endState(2);



    const auto a0 = s0;
    const auto a1 = s0_dot;
    const auto a2 = 0.5 * s0_dotdot;

    const auto T2 = T * T;
    const auto T3 = T2 * T;
    const auto T4 = T3 * T;

    MatrixXd A(2,2);
    A <<4 * T3, 3 * T2,
        12 * T2, 6 * T;

    VectorXd b(2);
    b << s1_dot -  a1  - 2 * a2 * T, s1_dotdot -  2 * a2;
    VectorXd C = A.colPivHouseholderQr().solve(b);
    const auto a3 = C(0);
    const auto a4 = C(1);
    VectorXd polycoeffs(5);

    polycoeffs << a0, a1, a2,a3, a4;

    return polycoeffs;

}

VectorXd CalcPositionAt(VectorXd polycoeffs, double time) {
    const auto a0 = polycoeffs(0);
    const auto a1 = polycoeffs(1);
    const auto a2 = polycoeffs(2);
    const auto a3 = polycoeffs(3);
    const auto a4 = polycoeffs(4);
    const auto a5 = polycoeffs(5);

    const auto t  = time;
    const auto t2 = t * t;
    const auto t3 = t2 * time;
    const auto t4 = t3 * time;
    const auto t5 = t4 * time;

    VectorXd position = VectorXd::Zero(3);
    position(0) = a0 + a1 * t + a2 * t2 + a3 * t3 + a4 * t4 + a5 * t5;
    position(1) = a1  + 2 * a2 * t + 3 * a3 * t2 + 4 * a4 * t3 + 5 * a5 * t4;
    position(2) = 2 * a2 + 3 * 2 * a3 * t + 12 * a4 * t2 + 20 * a5 * t3;

    return position;
}

VectorXd Trajectory::VelocityKeepingPath() {

}

FrenetPath Trajectory::VelocityKeepingPath() {

}

vector<FrenetPath> Trajectory::GetFrenetPaths(double current_speed, double current_d,
                                              double current_d_dot, double current_d_ddotdot, double s0) {
    vector<FrenetPath> fpaths;

    for (double roadPosition_d = 2; roadPosition_d <=8;  roadPosition_d + D_ROAD_W ){
        for (double timecounter = MINT; timecounter <= MAXT; timecounter + DT ) {
            vector<double> timesteps;
            vector<VectorXd> path_pos;
            for (double t = 0; t < timecounter; t += DT) {
                timesteps.push_back(t);
            }

            VectorXd startState(3);
            startState << current_d, current_d_dot, current_d_ddotdot;

            VectorXd endState(3);
            endState << roadPosition_d, 0, 0;

            VectorXd pathcoeffs = QuinicPolynomialCoeffs(startState, endState, timecounter);
            for (int t= 0; t < timesteps.size(); t++) {
                path_pos.push_back(CalcPositionAt(pathcoeffs, t));
            }
        }
    }


};


double Trajectory::CalcJerkAt(VectorXd coeffs, double time)
{
    // s(t) = a0 + a1 * t + a2 * t^2 + a3 * t^3 + a4 * t^4 + a5 * t^5
    //s_dot = a1 + 2a2 * t + 3ac * t^2 + 4a4 * t^4 + 5a5 * t^4
    // s_dotdot(t) = 2a2 + 6a3 * t + 12a4 * t^2 + 20a5 * t^3
    // calculate s_ddd(t)
    return 6 * coeffs(3) + 24 * coeffs(4) * time + 60 * coeffs(5) * time * time;
}

double Trajectory::CalcAccelAt(VectorXd coeffs, double time)
{
    // s(t) = a0 + a1 * t + a2 * t^2 + a3 * t^3 + a4 * t^4 + a5 * t^5
    //s_dot = a1 + 2a2 * t + 3ac * t^2 + 4a4 * t^4 + 5a5 * t^4
    // s_dotdot(t) = 2a2 + 6a3 * t + 12a4 * t^2 + 20a5 * t^3
    // calculate s_ddd(t)
    return 2 * coeffs(2) + 6 * coeffs(3) * time + 12 * coeffs(4) * time * time + 20 * coeffs(5) * time * time * time;
}

double Trajectory::JerkCost(double duration) {
    double cost_s = 0;
    double cost_d = 0;

    int steps = static_cast<int>(duration/ timestep);

    for (int i = 0; i < steps; i++) {
//        double jerk_s = CalcJerkAt()
    }
}


