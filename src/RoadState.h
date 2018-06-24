//
// Created by Wing Yee mak on 24/06/2018.
//

#ifndef PATH_PLANNING_ROADSTATE_H
#define PATH_PLANNING_ROADSTATE_H
#include "datastructs.h"
#include <vector>
#include <unordered_map>

using namespace std;

class RoadState {
public:
    RoadState();


    void CalcInViewCars();

    void UpdateRoadState(vector<SensorFusionData> sfData);

    void PredictRoadNextState();
private:
    unordered_map <int, SensorFusionData> mAllCars; //stashing all the cars from sensorfusion here
    unordered_map <int, SensorFusionData> mInviewCars; //only care about cars in front (think about too close behind cars??)
};


#endif //PATH_PLANNING_ROADSTATE_H
