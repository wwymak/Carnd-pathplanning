//
// Created by Wing Yee mak on 19/06/2018.
//

#ifndef PATH_PLANNING_OTHERVEHICLES_H
#define PATH_PLANNING_OTHERVEHICLES_H

#include "datastructs.h"
#include <vector>

using namespace std;

class OtherVehicles {
public:
    OtherVehicles();
    double duration;
    double timestep;
    vector<SensorFusionData> sensorFusionDataMap;
    vector<vector<CarPositonData>> predictedPaths;

    void setPredictionTimeSettings(double duration, double deltaT);
    void setSensorFusionData(vector<SensorFusionData> sfData);
    void setPredictedPaths();
    vector<vector<CarPositonData>> getPredictedPaths();
    vector<CarPositonData> CalcPredictedPath(SensorFusionData initPosition);
    vector<vector<CarPositonData>> CalcAllPredictedPath();


};


#endif //PATH_PLANNING_OTHERVEHICLES_H
