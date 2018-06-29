//
// Created by Wing Yee mak on 24/06/2018.
//

#ifndef PATH_PLANNING_ROADSTATE_H
#define PATH_PLANNING_ROADSTATE_H
#include "datastructs.h"
#include "waypoints.h"
#include "Utils.h"
#include <vector>
#include <unordered_map>

using namespace std;

class RoadState {
public:
    RoadState();


    void CalcInViewCars(CarPositonData& egoVehicle,  double deltaT);

    void UpdateRoadState(vector<SensorFusionData>& sfData, Waypoints& wps);

//    unordered_map <int, CarPositonData> GetInviewCars() {return mInviewCars;};
//    unordered_map <int, CarPositonData> GetInviewCarsLaneLeft() {return mInviewCarsLaneLeft;};
//    unordered_map <int, CarPositonData> GetInviewCarsLaneRight() {return mInviewCarsLaneRight;};

    void PredictRoadNextState();

    unordered_map <int, CarPositonData> mAllCars; //stashing all the cars from sensorfusion here
    unordered_map <int, CarPositonData> mInviewCars; //only care about cars in front (think about too close behind cars??)
    unordered_map <int, CarPositonData> mInviewCarsLaneLeft; //only care about cars in front (think about too close behind cars??)
    unordered_map <int, CarPositonData> mInviewCarsLaneRight; //only care about cars in front (think about too close behind cars??)
};


#endif //PATH_PLANNING_ROADSTATE_H
