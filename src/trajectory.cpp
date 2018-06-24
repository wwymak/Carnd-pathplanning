//
// Created by Wing Yee mak on 24/06/2018.
//

#include "trajectory.h"

using namespace std;
using namespace Eigen;

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


VectorXd Trajectory::QuinicPolynomialCoeffs(VectorXd startState, VectorXd endState, float T){

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