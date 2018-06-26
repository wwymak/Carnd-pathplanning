//
// Created by Wing Yee mak on 19/06/2018.
//

#include <iostream>
#include "OtherVehicles.h"
using namespace std;

OtherVehicles::OtherVehicles() {}

void OtherVehicles::setSensorFusionData(vector<SensorFusionData> sfData) {
    sensorFusionDataMap = sfData;
//    setPredictedPaths();
};

void OtherVehicles::setPredictedPaths() {
    predictedPaths = CalcAllPredictedPath();
};

vector<vector<CarPositonData>> OtherVehicles::getPredictedPaths() {
    return predictedPaths;
};
void OtherVehicles::setPredictionTimeSettings(double dur, double deltaT) {
    duration = dur;
    timestep = deltaT;
};

vector<CarPositonData> OtherVehicles::CalcPredictedPath(SensorFusionData initPosition) {
    vector<CarPositonData> predictedPath;
    double speed = sqrt(pow(initPosition.vx , 2) + pow(initPosition.vy, 2));
    double s0 = initPosition.s;
    for (int i = 0; i< int(timestep /duration); i++) {
        CarPositonData posInstance;
        posInstance.s = i * speed * timestep + s0;
        posInstance.d = initPosition.d;

        predictedPath.push_back(posInstance);
    }
    return predictedPath;
};

vector<vector<CarPositonData>> OtherVehicles::CalcAllPredictedPath() {
    vector<vector<CarPositonData>> allOtherVehicles;
    for (int i = 0; i< sensorFusionDataMap.size(); i++) {
        SensorFusionData initPosition = sensorFusionDataMap.at(i);
//        cout << "ionit position"<< endl;
//        cout<< initPosition.x<< "," << initPosition.y << ","<< initPosition.s << endl;
        vector<CarPositonData> pathPoints = CalcPredictedPath(initPosition);
        allOtherVehicles.push_back(pathPoints);
    }
    cout << "here"<< allOtherVehicles.size()<< endl;
    return allOtherVehicles;
};

//flattne the PredictedPaths vector of all other vehicles into a vector of obstacles that the
//vector<double> OtherVehicles::ConvertPathsToObstacles() {
//    vector<double> obstacles;
//    for (int i = 0; i< sensorFusionDataMap.size(); i++) {
//        SensorFusionData initPosition = sensorFusionDataMap.at(i);
//        vector<CarPositonData> pathPoints = CalcPredictedPath(initPosition);
//        allOtherVehicles.push_back(pathPoints);
//    }
//    return obstacles;
//};
