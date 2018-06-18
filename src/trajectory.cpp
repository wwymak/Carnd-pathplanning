#include "util.h"
#include "trajectory.h"
#include "datastructs.h"
#include <iostream>
#include <numeric>
#include "algorithm"
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
const double MAX_ROAD_D = 8.0;   //maximum road width [m]
const double MIN_ROAD_D = 2.0;   //maximum road width [m]
const double D_ROAD_W = 1.0;   // road width sampling length [m]
const double DT = 0.02;   // time tick [s]
const double MAXT = 5.0;   // max prediction time [s]
const double MINT = 2.0;  // min prediction time [s]
const double TARGET_SPEED = mileph2meterps(49);   // [m/s]
const double MIN_TARGET_SPEED = mileph2meterps(40);   // [m/s]
const double D_T_S = mileph2meterps(0.5);   //target speed sampling length [m/s]
const double N_S_SAMPLE = 1 ;  // sampling number of target speed


// cost weights
const double KJ = 0.1;
const double KT = 0.1;
const double KD = 1.0;
const double KS = 1.0;
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

    const auto s1_dot = endState(0);
    const auto s1_dotdot = endState(1);



    const auto a0 = s0;
    const auto a1 = s0_dot;
    const auto a2 = 0.5 * s0_dotdot;

    const auto T2 = T * T;
    const auto T3 = T2 * T;

    MatrixXd A(2,2);
    A <<4 * T3, 3 * T2,
        12 * T2, 6 * T;

    VectorXd b(2);
    b << s1_dot -  a1  - 2 * a2 * T, s1_dotdot -  2 * a2;
    VectorXd C = A.colPivHouseholderQr().solve(b);
    const auto a3 = C(0);
    const auto a4 = C(1);
    VectorXd polycoeffs(6);

    polycoeffs << a0, a1, a2,a3, a4, 0;

    return polycoeffs;

}

VectorXd Trajectory::CalcPositionAt(VectorXd polycoeffs, double time) {
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
    position(3) = 3 * 2 * a3  + 24 * a4 * t + 60 * a5 * t2;

    return position;
}

vector<FrenetPath> Trajectory::GetFrenetPaths(double current_s,double current_s_dot, double current_s_dotdot,  double current_d,
                                              double current_d_dot, double current_d_ddotdot) {
    vector<FrenetPath> fpaths;

    for (int road_d_idx = 0; road_d_idx< int((MAX_ROAD_D - MIN_ROAD_D)/ D_ROAD_W) ; road_d_idx++) {
        double roadPosition_d = road_d_idx * D_ROAD_W + MIN_ROAD_D;

        for (int timecounter = 0; timecounter < int((MAXT - MINT)/DT); timecounter ++) {
            double predDuration = timecounter * DT + MINT;


            vector<double> timesteps;
            vector<VectorXd> path_pos;

            for (int i = 0; i< int(predDuration / DT); i++) {
                timesteps.push_back(i * DT);
            }


            VectorXd startState(3);
            startState << current_d, current_d_dot, current_d_ddotdot;
            VectorXd endState(3);
            endState << roadPosition_d, 0, 0;


            VectorXd pathcoeffs = QuinicPolynomialCoeffs(startState, endState, predDuration);
            for (int t= 0; t < timesteps.size(); t++) {
                path_pos.push_back(CalcPositionAt(pathcoeffs, t));
            }
            vector<double> d (path_pos.size());
            vector<double> d_dot (path_pos.size());
            vector<double> d_dotdot (path_pos.size());
            vector<double> d_dotdotdot(path_pos.size());


            for (int i = 0; i< path_pos.size(); i++) {
                d[i] = path_pos[i](0);
                d_dot[i] = path_pos[i](1);
                d_dotdot[i] = path_pos[i](2);
                d_dotdotdot[i] = path_pos[i](3);
            }


            //longitudinal veloctiy keeping

            for (int i = 0; i< int((TARGET_SPEED - MIN_TARGET_SPEED)/D_T_S) ; i++) {
                double target_s = MIN_TARGET_SPEED + i * D_T_S;



                vector<VectorXd> path_pos_s;

                VectorXd startState_s(3);
                startState_s << current_s, current_s_dot, 0;

                VectorXd endState_s(2);
                endState_s << target_s, 0;

                VectorXd pathcoeffs_long = QuarticPolynomialCoeffs(startState_s, endState_s, predDuration);
                for (int t= 0; t < timesteps.size(); t++) {
                    path_pos_s.push_back(CalcPositionAt(pathcoeffs_long, t));
                }

                vector<double> s (path_pos_s.size());
                vector<double> s_dot (path_pos_s.size());
                vector<double> s_dotdot (path_pos_s.size());
                vector<double> s_dotdotdot(path_pos_s.size());


                for (int i = 0; i< path_pos.size(); i++) {
                    s[i] = path_pos_s[i](0);
                    s_dot[i] = path_pos_s[i](1);
                    s_dotdot[i] = path_pos_s[i](2);
                    s_dotdotdot[i] = path_pos_s[i](3);
                }

                FrenetPath fp;

                fp.timesteps = timesteps;
                fp.d = d;
                fp.d_dot = d_dot;
                fp.d_dotdot = d_dotdot;
                fp.d_dotdotdot = d_dotdotdot;
                fp.s = s;
                fp.s_dot = s_dot;
                fp.s_dotdot = s_dotdot;
                fp.s_dotdotdot = s_dotdotdot;

                vector<size_t> jerk_s;
                vector<size_t> jerk_d;
                transform(s_dotdotdot.begin(), s_dotdotdot.end(), jerk_s.begin(), [](double dotdotdot){
                    if(dotdotdot > MAX_JERK) {
                        return 1000 * dotdotdot * dotdotdot;
                    }
                    return dotdotdot * dotdotdot;
                });
                transform(d_dotdotdot.begin(), d_dotdotdot.end(), jerk_d.begin(), [](double dotdotdot){
                    if(dotdotdot > MAX_JERK) {
                        return 1000 * dotdotdot * dotdotdot;
                    }
                    return dotdotdot * dotdotdot;
                });

                double cost_jerk_s = accumulate(jerk_s.begin(), jerk_s.end(),0.0);
                double cost_jerk_d = accumulate(jerk_d.begin(), jerk_d.end(),0.0);


                double diff_s_dot_cost = pow((TARGET_SPEED - s_dot[s_dot.size()-1]), 2);
                //heavily penalise paths that exceeds speed limit
                if(MAX_SPEED < s_dot[s_dot.size()-1]) {
                    diff_s_dot_cost = 10000;
                }
                double diff_d_dot_cost = pow(d_dot[d_dot.size()-1], 2);

                double cost_v =  KJ * cost_jerk_s + KT * predDuration + KS * diff_s_dot_cost;
                double cost_d =  KJ * cost_jerk_d + KT * predDuration + KD * diff_s_dot_cost;

                fp.cost_d = cost_d;
                fp.cost_s = cost_v;
                fp.cost_total = KLAT * cost_d + KLON *  cost_v;


                fpaths.push_back(fp);
            }


        }
    }

    return fpaths;
};

//todo not quite correct yet
vector<FrenetPath> Trajectory::GetFrenetPathsSpeedChanging(double current_s,double current_s_dot, double current_s_dotdot,  double current_d,
                                              double current_d_dot, double current_d_ddotdot) {
    vector<FrenetPath> fpaths;

    for (int road_d_idx = 0; road_d_idx< int((MAX_ROAD_D - MIN_ROAD_D)/ D_ROAD_W) ; road_d_idx++) {
        double roadPosition_d = road_d_idx * D_ROAD_W + MIN_ROAD_D;

        for (int timecounter = 0; timecounter < int((MAXT - MINT)/DT); timecounter ++) {
            double predDuration = timecounter * DT + MINT;


            vector<double> timesteps;
            vector<VectorXd> path_pos;

            for (int i = 0; i< int(predDuration / DT); i++) {
                timesteps.push_back(i * DT);
            }


            VectorXd startState(3);
            startState << current_d, current_d_dot, current_d_ddotdot;
            VectorXd endState(3);
            endState << roadPosition_d, 0, 0;


            VectorXd pathcoeffs = QuinicPolynomialCoeffs(startState, endState, predDuration);
            for (int t= 0; t < timesteps.size(); t++) {
                path_pos.push_back(CalcPositionAt(pathcoeffs, t));
            }
            vector<double> d (path_pos.size());
            vector<double> d_dot (path_pos.size());
            vector<double> d_dotdot (path_pos.size());
            vector<double> d_dotdotdot(path_pos.size());


            for (int i = 0; i< path_pos.size(); i++) {
                d[i] = path_pos[i](0);
                d_dot[i] = path_pos[i](1);
                d_dotdot[i] = path_pos[i](2);
                d_dotdotdot[i] = path_pos[i](3);
            }


            //longitudinal veloctiy keeping

            for (int i = 0; i< int((TARGET_SPEED - MIN_TARGET_SPEED)/D_T_S) ; i++) {
                double target_s = MIN_TARGET_SPEED + i * D_T_S;



                vector<VectorXd> path_pos_s;

                VectorXd startState_s(3);
                startState_s << current_s, current_s_dot, 0;

                VectorXd endState_s(2);
                endState_s << target_s, 0;

                VectorXd pathcoeffs_long = QuarticPolynomialCoeffs(startState_s, endState_s, predDuration);
                for (int t= 0; t < timesteps.size(); t++) {
                    path_pos_s.push_back(CalcPositionAt(pathcoeffs_long, t));
                }

                vector<double> s (path_pos_s.size());
                vector<double> s_dot (path_pos_s.size());
                vector<double> s_dotdot (path_pos_s.size());
                vector<double> s_dotdotdot(path_pos_s.size());


                for (int i = 0; i< path_pos.size(); i++) {
                    s[i] = path_pos_s[i](0);
                    s_dot[i] = path_pos_s[i](1);
                    s_dotdot[i] = path_pos_s[i](2);
                    s_dotdotdot[i] = path_pos_s[i](3);
                }

                FrenetPath fp;

                fp.timesteps = timesteps;
                fp.d = d;
                fp.d_dot = d_dot;
                fp.d_dotdot = d_dotdot;
                fp.d_dotdotdot = d_dotdotdot;
                fp.s = s;
                fp.s_dot = s_dot;
                fp.s_dotdot = s_dotdot;
                fp.s_dotdotdot = s_dotdotdot;

                vector<size_t> jerk_s;
                vector<size_t> jerk_d;
                transform(s_dotdotdot.begin(), s_dotdotdot.end(), jerk_s.begin(), [](double dotdotdot){
                    if(dotdotdot > MAX_JERK) {
                        return 1000 * dotdotdot * dotdotdot;
                    }
                    return dotdotdot * dotdotdot;
                });
                transform(d_dotdotdot.begin(), d_dotdotdot.end(), jerk_d.begin(), [](double dotdotdot){
                    if(dotdotdot > MAX_JERK) {
                        return 1000 * dotdotdot * dotdotdot;
                    }
                    return dotdotdot * dotdotdot;
                });

                double cost_jerk_s = accumulate(jerk_s.begin(), jerk_s.end(),0.0);
                double cost_jerk_d = accumulate(jerk_d.begin(), jerk_d.end(),0.0);


                double diff_s_dot_cost = pow((TARGET_SPEED - s_dot[s_dot.size()-1]), 2);
                //heavily penalise paths that exceeds speed limit
                if(MAX_SPEED < s_dot[s_dot.size()-1]) {
                    diff_s_dot_cost = 10000;
                }
                double diff_d_dot_cost = pow(d_dot[d_dot.size()-1], 2);

                double cost_v =  KJ * cost_jerk_s + KT * predDuration + KS * diff_s_dot_cost;
                double cost_d =  KJ * cost_jerk_d + KT * predDuration + KD * diff_s_dot_cost;

                fp.cost_d = cost_d;
                fp.cost_s = cost_v;
                fp.cost_total = KLAT * cost_d + KLON *  cost_v;


                fpaths.push_back(fp);
            }


        }
    }

    return fpaths;
};







