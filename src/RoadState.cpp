//
// Created by Wing Yee mak on 24/06/2018.
//

#include "RoadState.h"

void RoadState::UpdateRoadState(vector<SensorFusionData> sfData, Waypoints wps) {
    for (int i = 0; i< sfData.size(); i++) {
        SensorFusionData sfd = sfData.at(i);
        int id = sfd.id;
        double speed = sqrt(sfd.x * sfd.x + sfd.y + sfd.y);
        CarPositonData c;
        c.speed = speed;
        c.id = id;
        c.s = sfd.s;
        c.d = sfd.d;

        if (c.s >= wps.max_track_s) {
            c.s -= wps.max_track_s;
        }

        mAllCars[id] =c;
    }


}

void RoadState::CalcInViewCars(double ego_s, double ego_d, double horizon) {

};

