//
// Created by Wing Yee mak on 23/06/2018.
//

#include "LaneFSM.h"

FrenetPath LaneFSM::OptimalPath(VectorXd currentState, double currentTime, OtherVehicles otherVehicles) {

}

int LaneFSM::CanChangeLane() {
    unordered_map <int, CarPositonData> mInviewCarsLaneLeft = rs.GetInviewCarsLaneLeft();
    unordered_map <int, CarPositonData> mInviewCarsLaneRight = rs.GetInviewCarsLaneRight();

    if(mInviewCarsLaneLeft.size() == 0){
        return -1;
    }else if(mInviewCarsLaneRight.size() == 0){
        return 1;
    } else {
        return 0;
    }
}