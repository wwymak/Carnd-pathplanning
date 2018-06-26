//
// Created by Wing Yee mak on 26/06/2018.
//

#ifndef PATH_PLANNING_UTILS_H
#define PATH_PLANNING_UTILS_H


class Utils {
public:
    const double MAX_SPEED = 49.5;  // maximum speed [mileph]
    const double MAX_ACCEL = 10.0 ; // maximum acceleration [m/ss]
    const double MAX_JERK = 10.0 ; // maximum acceleration [m/sss]
    const double MAX_CURVATURE = 1.0;;   // maximum curvature [1/m]
    const double MAX_ROAD_D = 8.0;   //maximum road width [m]
    const double MIN_ROAD_D = 2.0;   //maximum road width [m]
    const double D_ROAD_W = 1.0;   // road width sampling length [m]
    const double DT = 0.02;   // time tick [s]
    const double MAXT = 2.0;   // max prediction time [s]
    const double MINT = 0.8;  // min prediction time [s]
    const double TARGET_SPEED = 49;   // [m/s]
    const double MIN_TARGET_SPEED = 40;   // [m/s]
    const double D_T_S = 0.5;   //target speed sampling length [m/s]
    const double N_S_SAMPLE = 1 ;  // sampling number of target speed
    const double SAFE_DISTANCE = MAX_SPEED * 2; //aka the 2 second rule
};


#endif //PATH_PLANNING_UTILS_H
