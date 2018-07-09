//
// Created by Wing Yee mak on 24/06/2018.
//

#include "RoadState.h"

RoadState::RoadState() {};

void RoadState::UpdateRoadState(vector<SensorFusionData>& sfData, Waypoints& wps) {
    mAllCars.clear();
    for (int i = 0; i< sfData.size(); i++) {
        SensorFusionData sfd = sfData.at(i);
        int id = sfd.id;
        double speed = sqrt(sfd.vx * sfd.vx + sfd.vy + sfd.vy);
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

void RoadState::CalcInViewCars(CarPositonData& egoVehicle, double deltaT) {
    mInviewCars.clear();
    mInviewCarsLaneLeft.clear();
    mInviewCarsLaneRight.clear();
    int egoLane = ConvertDToLane(egoVehicle.d);
//    double egoHorizonS = 50 + egoVehicle.s;
    double egoHorizonS = egoVehicle.s +  egoVehicle.speed * deltaT;
    for ( auto it = mAllCars.begin(); it != mAllCars.end(); ++it ) {
        int id = it->first;
        CarPositonData otherCar = it->second;
        double otherCar_S_Horizon = otherCar.s + otherCar.speed * deltaT;
        int otherCarD =ConvertDToLane(otherCar.d);
//        cout << "other car d"<< otherCar.d<< endl;
        if (otherCarD < 0 || otherCarD > 12) {
            continue;
        }
        if((otherCar.s > egoVehicle.s) &&  (egoHorizonS >otherCar_S_Horizon)) {
            if(otherCarD == egoLane) {
                mInviewCars[id] = otherCar;
            } else if(otherCarD - egoLane == 1) {
                mInviewCarsLaneRight[id] = otherCar;
            } else if(otherCarD - egoLane == -1) {
                mInviewCarsLaneLeft[id] = otherCar;
            }
        }
        if((otherCar.s <= egoVehicle.s) &&  (egoHorizonS <otherCar_S_Horizon)) {
            if(otherCarD - egoLane == 1) {
                mInviewCarsLaneRight[id] = otherCar;
            } else if(otherCarD - egoLane == -1) {
                mInviewCarsLaneLeft[id] = otherCar;
            }
        }
//        if(((otherCar.s > egoVehicle.s) &&  (egoHorizonS >otherCar_S_Horizon)) || ((otherCar.s <= egoVehicle.s) &&  (egoHorizonS <otherCar_S_Horizon))) {
//            if(otherCarD == egoLane) {
//                mInviewCars[id] = otherCar;
//            } else if(otherCarD - egoLane == 1) {
//                mInviewCarsLaneRight[id] = otherCar;
//            } else if(otherCarD - egoLane == -1) {
//                mInviewCarsLaneLeft[id] = otherCar;
//            }
//        }
    }
//
//    cout << "cars in front:" << mInviewCars.size()<< endl;
//    cout << "cars roght:" << mInviewCarsLaneRight.size()<< endl;
//    cout << "cars left:" << mInviewCarsLaneLeft.size()<< endl;
};

