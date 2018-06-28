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
    unordered_map <int, CarPositonData> mInviewCarsLaneLeft = rs.GetInviewCarsLaneLeft();
    unordered_map <int, CarPositonData> mInviewCarsLaneRight = rs.GetInviewCarsLaneRight();

    cout << mInviewCarsLaneLeft.size() <<","<< mInviewCarsLaneRight.size()<< endl;
    if(mInviewCarsLaneLeft.empty() && mInviewCarsLaneRight.empty()){
        return 3;
    }else if(mInviewCarsLaneLeft.empty() == 0){
        return 2;
    }else if(mInviewCarsLaneRight.empty() == 0){
        return 1;
    } else {
        return 0;
    }
}