//
// Created by Wing Yee mak on 19/06/2018.
//

#ifndef PATH_PLANNING_OTHERVEHICLES_H
#define PATH_PLANNING_OTHERVEHICLES_H

#include "datastructs.h";
#include <vector>

using namespace std;

class OtherVehicles {
public:
    double duration;
    double timestep;
    vector<SensorFusionData> sensorFusionDataMap;
    void setSensorFusionData(vector<SensorFusionData> sfData);
    vector<vector<double>> GetPredictedPath(SensorFusionData initPosition, double timeduration, double deltaT);
    vector<vector<double>> GetAllPredictedPath(SensorFusionData initPosition, double timeduration, double deltaT);


};


#endif //PATH_PLANNING_OTHERVEHICLES_H
