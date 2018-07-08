//
// Created by Wing Yee mak on 24/06/2018.
//

#include "trajectory.h"

using namespace std;
using namespace Eigen;

Trajectory::Trajectory() {};

Trajectory::Trajectory(const VectorXd &startState, const VectorXd &endState, double durationS, double durationD,
                       double timeStart):
    mTimeStart(timeStart),
    mDurationS(durationS),
    mDurationD(durationD),
    mTargetSpeed(0),
    mStartState(startState),
    mEndState(endState),
    mSCoeffs(VectorXd::Zero(6)),
    mDCoeffs(VectorXd::Zero(6))
{
    mSCoeffs = QuinicPolynomialCoeffs(startState.head(3), endState.head(3), durationS);
    mDCoeffs = QuinicPolynomialCoeffs(startState.tail(3), endState.tail(3), durationD);
}


VectorXd Trajectory::QuinicPolynomialCoeffs(VectorXd startState, VectorXd endState, double T){

    const auto s0 = startState(0);
    const auto s0_dot = startState(1);
    const auto s0_dotdot = startState(2);

    const auto s1 = endState(0);
    const auto s1_dot = endState(1);
    const auto s1_dotdot = endState(2);

//    cout << "QuinicPolynomialCoeffs" << s0 << "," << s0_dot << "," << s0_dotdot<< ","<< s1<< ","<< s1_dot << "," << s1_dotdot<< endl;

    const auto a0 = s0;
    const auto a1 = s0_dot;
    const auto a2 = 0.5 * s0_dotdot;

    const auto T2 = T * T;
    const auto T3 = T2 * T;
    const auto T4 = T3 * T;
    const auto T5 = T4 * T;

    MatrixXd A(3, 3);

    A << T3, T4, T5,
            3 * T2, 4 * T3, 5 * T4,
            6 * T, 12 * T2, 20 * T3;

    VectorXd b(3);
    b << s1 - (a0 + a1 * T + a2 * T2), s1_dot - (a1 + 2 * a2 * T) , s1_dotdot - 2 * a2;
    VectorXd C = A.colPivHouseholderQr().solve(b);
    const auto a3 = C(0);
    const auto a4 = C(1);
    const auto a5 = C(2);

    VectorXd polycoeffs(6);
    polycoeffs << a0, a1, a2,a3, a4, a5;
    return polycoeffs;


}

VectorXd Trajectory::QuarticPolynomialCoeffs(VectorXd startState, VectorXd endState, double T){

    const auto s0 = startState(0);
    const auto s0_dot = startState(1);
    const auto s0_dotdot = startState(2);

    const auto s1_dot = endState(0);
    const auto s1_dotdot = endState(1);
//
//    cout << "QuarticPolynomialCoeffs" << s0 << "," << s0_dot << "," << s0_dotdot<< ","<< s1_dot << "," << s1_dotdot<< endl;


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
//    cout << "Quartic solver start" << endl;
    VectorXd C = A.colPivHouseholderQr().solve(b);
    const auto a3 = C(0);
    const auto a4 = C(1);
    VectorXd polycoeffs(6);
//    cout << "Quartic solver end" << a0<< "," << a1<< "," << a2<< endl;
    polycoeffs << a0, a1, a2,a3, a4, 0;

    return polycoeffs;

}

FrenetTraj Trajectory::TrajectoryS_Highspeed(VectorXd startState, VectorXd endState, double targetSpeed,
                                             double currentTime, double durationS, double durationD) {
    FrenetTraj fTraj;
    VectorXd endForFitting(2);
    endForFitting << targetSpeed, 0;

    fTraj.sCoeffs = QuarticPolynomialCoeffs(startState,endForFitting , durationS);
    fTraj.dCoeffs = QuinicPolynomialCoeffs(startState.tail(3), endState.tail(3), durationD);
    fTraj.startState = startState;
    fTraj.endState = endState;
    fTraj.durationS = durationS;
    fTraj.durationD = durationD;

    return fTraj;
}

vector<FrenetPath> Trajectory::GetFrenetPaths(double current_s,double current_s_dot, double current_s_dotdot,  double current_d,
                                              double current_d_dot, double current_d_ddotdot,double  target_d) {
    vector<FrenetPath> fpaths;

    for (int road_d_idx = 0; road_d_idx< int((MAX_ROAD_D - MIN_ROAD_D)/ D_ROAD_W) ; road_d_idx++) {
        double roadPosition_d = road_d_idx * D_ROAD_W + MIN_ROAD_D;

        double predDuration = MAXT;


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
//            cout<< "here"<< pathcoeffs.size()<< endl;
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

        for (int i = 0; i< min(30, int(abs(TARGET_SPEED - current_s_dot)/D_T_S)) ; i++) {
            double target_s = current_s_dot;

            if(current_s_dot < TARGET_SPEED) {
                target_s  += i * D_T_S;
            } else if (current_s_dot > TARGET_SPEED) {
                target_s  -= i * D_T_S;
            }


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
//                cout << "here3, construct fp"<< endl;
            vector<double> jerk_s(s_dotdotdot.size());
            vector<double> jerk_d(s_dotdotdot.size());
            transform(s_dotdotdot.begin(), s_dotdotdot.end(), jerk_s.begin(), [this](double dotdotdot){
                if(dotdotdot > MAX_JERK) {
                    return 1000 * dotdotdot * dotdotdot;
                }
                return dotdotdot * dotdotdot;
            });
            transform(d_dotdotdot.begin(), d_dotdotdot.end(), jerk_d.begin(), [this](double dotdotdot){
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
            double diff_d_cost = pow((target_d - d[d.size()-1]), 2);

            double cost_v =  KJ * cost_jerk_s + KT * predDuration + KS * diff_s_dot_cost;
            double cost_d =  KJ * cost_jerk_d + KT * predDuration + KD * diff_d_cost;

            fp.cost_d = cost_d;
            fp.cost_s = cost_v;
            fp.cost_total = KLAT * cost_d + KLON *  cost_v;

            fpaths.push_back(fp);
        }
    }

    return fpaths;
};

vector<FrenetPath>  Trajectory::GetSingleFrenetPaths(double current_s,double current_s_dot, double current_s_dotdot,  double current_d,
                                       double current_d_dot, double current_d_ddotdot,double  target_d) {
    vector<FrenetPath> fpaths;

//    for (int road_d_idx = 0; road_d_idx< int((MAX_ROAD_D - MIN_ROAD_D)/ D_ROAD_W) ; road_d_idx++) {
        double roadPosition_d = target_d;

        double predDuration = MAXT;


        vector<double> timesteps;
        vector<VectorXd> path_pos;

        for (int i = 0; i< int(predDuration / DT); i++) {
            timesteps.push_back(i * DT);
        }


        VectorXd startState(3);
        startState << current_d, current_d_dot, current_d_ddotdot;
        VectorXd endState(3);
//        endState << roadPosition_d, 0, 0;
        endState << current_d, 0, 0;


        VectorXd pathcoeffs = QuinicPolynomialCoeffs(startState, endState, predDuration);
//            cout<< "here"<< pathcoeffs.size()<< endl;
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

        for (int i = 1; i< min(30, int(abs(TARGET_SPEED - current_s_dot)/D_T_S)) ; i++) {
            double target_s_dot = current_s_dot;
            cout << "target_s_dot"<< target_s_dot<< endl;
            cout << "current_s_dot"<< current_s_dot<< endl;

            if(current_s_dot < TARGET_SPEED) {
                target_s_dot  += i * D_T_S;
            } else if (current_s_dot > TARGET_SPEED) {
                target_s_dot  -= i * D_T_S;
            }

            cout << "target_s_dot"<< target_s_dot<< endl;


            vector<VectorXd> path_pos_s;

            VectorXd startState_s(3);
            startState_s << current_s, current_s_dot, 0;

            VectorXd endState_s(2);
            endState_s << target_s_dot, 0;

//            VectorXd endState_s(3);
//            endState_s << target_s_dot, 0, 0;

//            VectorXd pathcoeffs_long = QuinicPolynomialCoeffs(startState_s, endState_s, predDuration);
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
//                cout << "here3, construct fp"<< endl;
            vector<double> jerk_s(s_dotdotdot.size());
            vector<double> jerk_d(s_dotdotdot.size());
            transform(s_dotdotdot.begin(), s_dotdotdot.end(), jerk_s.begin(), [this](double dotdotdot){
                if(dotdotdot > MAX_JERK) {
                    return 1000 * dotdotdot * dotdotdot;
                }
                return dotdotdot * dotdotdot;
            });
            transform(d_dotdotdot.begin(), d_dotdotdot.end(), jerk_d.begin(), [this](double dotdotdot){
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
            double diff_d_cost = pow((target_d - d[d.size()-1]), 2);

            double cost_v =  KJ * cost_jerk_s + KT * predDuration + KS * diff_s_dot_cost;
            double cost_d =  KJ * cost_jerk_d + KT * predDuration + KD * diff_d_cost;

            fp.cost_d = cost_d;
            fp.cost_s = cost_v;
            fp.cost_total = KLAT * cost_d + KLON *  cost_v;

            cout << "cost diff s" << diff_s_dot_cost<< endl;

            fpaths.push_back(fp);

        }

        sort(fpaths.begin(), fpaths.end(), [](FrenetPath a, FrenetPath b) {
            return (a.cost_total < b.cost_total );
        });

    return fpaths;
};

VectorXd Trajectory::CalcPositionAt(VectorXd polycoeffs, double time) {
    const auto a0 = polycoeffs(0);
    const auto a1 = polycoeffs(1);
    const auto a2 = polycoeffs(2);
    const auto a3 = polycoeffs(3);
    const auto a4 = polycoeffs(4);
    const auto a5 = polycoeffs(5);

    const auto t  = time;
    const auto t2 = t * t;
    const auto t3 = t2 * t;
    const auto t4 = t3 * t;
    const auto t5 = t4 * t;

    VectorXd position = VectorXd::Zero(4);
    position(0) = a0 + a1 * t + a2 * t2 + a3 * t3 + a4 * t4 + a5 * t5;
    position(1) = a1  + 2 * a2 * t + 3 * a3 * t2 + 4 * a4 * t3 + 5 * a5 * t4;
    position(2) = 2 * a2 + 3 * 2 * a3 * t + 12 * a4 * t2 + 20 * a5 * t3;
    position(3) = 3 * 2 * a3  + 24 * a4 * t + 60 * a5 * t2;

    return position;
}

TrajectoryStandard Trajectory::CalcSplineTraj(TrajectoryStandard &previous_xy, CarPositonData &car, Waypoints &wps, double targetSpeed, int targetLane) {
    TrajectoryStandard outputSplineTraj;
    double horizon_x = 30.0;
    bool too_close = false;
    double too_close_other_car_s = 100;

    vector<double> previous_path_x = previous_xy.x_pts;
    vector<double> previous_path_y = previous_xy.y_pts;
    int prevpath_size = previous_path_x.size();

    vector<double> next_x_vals;
    vector<double> next_y_vals;
    vector<double> pts_x;
    vector<double> pts_y;

    double ref_x = car.x;
    double ref_y = car.y;
    double ref_yaw = deg2rad(car.yaw);
    double ref_s = car.s;

    double ref_x_prev = car.x - cos(ref_yaw);
    double ref_y_prev = car.y - sin(ref_yaw);

    if(ref_yaw > deg2rad(25)) {
        ref_yaw = deg2rad(25);
    }

    if (prevpath_size >=  2) {
        ref_x = previous_path_x[prevpath_size -1];
        ref_y = previous_path_y[prevpath_size -1];

        ref_x_prev = previous_path_x[prevpath_size -2];
        ref_y_prev = previous_path_y[prevpath_size -2];

        ref_yaw = atan2((ref_y - ref_y_prev), (ref_x - ref_x_prev));
        ref_s = previous_xy.path_end_s;

    }

    pts_x.push_back(ref_x_prev);
    pts_x.push_back(ref_x);

    pts_y.push_back(ref_y_prev);
    pts_y.push_back(ref_y);


    // getting a largely spaced waypoints to fit spline
    for (int i = 1; i < 4; i++) {
        vector<double> next_wp= wps.getXYSimple((ref_s + (i * 30)), (2 + 4 * targetLane));
        pts_x.push_back(next_wp[0]);
        pts_y.push_back(next_wp[1]);
    }
        vector<double> fitpts_x;
        vector<double> fitpts_y;
        for (int i = 0; i< pts_x.size(); i++) {
            double shifted_x = pts_x.at(i) - ref_x;
            double shifted_y = pts_y.at(i) - ref_y;
            double converted_x = shifted_x * cos(0 - ref_yaw) - shifted_y * sin(0 -ref_yaw);
            double converted_y = shifted_x * sin(0 - ref_yaw) + shifted_y * cos(0 -ref_yaw);
            if (i == 0 || converted_x > fitpts_x.at(i -1)){
                fitpts_x.push_back(converted_x);
                fitpts_y.push_back(converted_y);
            } else {
                cout << "weird points!?" << converted_x << ","<< fitpts_x.at(i -1)<< endl;
            }
        }

        tk::spline s;
        //set anchor points for spline
        s.set_points(fitpts_x, fitpts_y);

        //add points remaining from previous path (path planning points)
        for (int i = 0; i < prevpath_size; i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
        }



        double horizon_y = s(horizon_x);

        double target_d =  sqrt(horizon_x *  horizon_x + horizon_y * horizon_y);
        double x_sum_prev = 0; // the distance in x covered by previous points

        for (int i = 0; i< 50 - prevpath_size; i++) {
            double N = target_d / ( targetSpeed * 0.02 / 2.24);
            double x_point = x_sum_prev + horizon_x / N;
            double y_point = s(x_point);

            double x_ref = x_point;
            double y_ref = y_point;

            x_sum_prev = x_point;

            x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));


            next_x_vals.push_back(x_point + ref_x);
            next_y_vals.push_back(y_point + ref_y);
        }

    outputSplineTraj.x_pts = next_x_vals;
    outputSplineTraj.y_pts = next_y_vals;

    return outputSplineTraj;
}