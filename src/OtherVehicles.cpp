//
// Created by Wing Yee mak on 19/06/2018.
//

#include "OtherVehicles.h"


OtherVehicles::OtherVehicles() {}

void OtherVehicles::setSensorFusionData(vector<SensorFusionData> sfData) {
    sensorFusionDataMap = sfData;

};

void OtherVehicles::setPredictionTimeSettings(double dur, double deltaT) {
    duration = dur;
    timestep = deltaT;
};
vector<vector<double>> OtherVehicles::GetPredictedPath(SensorFusionData initPosition, double timeduration, double deltaT) {
    vector<vector<double>> predictedPath;
    double speed = sqrt(pow(initPosition.vx , 2) + pow(initPosition.vy, 2));
    double s0 = initPosition.s;
    for (int i = 0; i< int(timeduration /deltaT); i++) {
        double s = i * speed * deltaT + s0;
        double d = initPosition.d;

        predictedPath.push_back({s, d});
    }
    return predictedPath;
};

vector<vector<double>> GetAllPredictedPath(SensorFusionData initPosition, double timeduration, double deltaT) {
//    for (int)
};
