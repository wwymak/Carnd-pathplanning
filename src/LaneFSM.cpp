//
// Created by Wing Yee mak on 23/06/2018.
//

#include "LaneFSM.h"
//
//FrenetPath LaneFSM::OptimalPath(VectorXd currentState, double currentTime, OtherVehicles otherVehicles) {
//
//}
LaneFSM::LaneFSM(RoadState& rs) :rs(rs) {}

int LaneFSM::CanChangeLane() {
    unordered_map <int, CarPositonData> mInviewCarsLaneLeft = rs.mInviewCarsLaneLeft;
    unordered_map <int, CarPositonData> mInviewCarsLaneRight = rs.mInviewCarsLaneRight;

    cout<< "can change lane " << mInviewCarsLaneLeft.size() <<","<< mInviewCarsLaneRight.size()<< endl;
    if(mInviewCarsLaneLeft.empty() && mInviewCarsLaneRight.empty()){
        return 3;
    }else if(mInviewCarsLaneLeft.empty()){
        return 2;
    }else if(mInviewCarsLaneRight.empty()){
        return 1;
    } else {
        return 0;
    }
}

void LaneFSM::LaneKeeping(CarPositonData& egoVehicle) {
    unordered_map <int, CarPositonData> mInviewCarsFront = rs.mInviewCars;
    VectorXd egoStartState(6);
    VectorXd egoEndState(6);
    egoStartState << egoVehicle.s, egoVehicle.speed, 0, egoVehicle.d, 0,0;
    if(mInviewCarsFront.size() >0) {
        //todo generate lane changing
    } else {
        //todo lanekeeping
        //todo generate lane keeping path
    }
}