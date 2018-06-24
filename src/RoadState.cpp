//
// Created by Wing Yee mak on 24/06/2018.
//

#include "RoadState.h"

void RoadState::UpdateRoadState(vector<SensorFusionData> sfData) {
    for (int i = 0; i< sfData.size(); i++) {
        int id = sfData.at(i).id;

        mAllCars[id] = sfData.at(i);
    }


}