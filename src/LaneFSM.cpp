//
// Created by Wing Yee mak on 23/06/2018.
//

#include "LaneFSM.h"
//
//FrenetPath LaneFSM::OptimalPath(VectorXd currentState, double currentTime, OtherVehicles otherVehicles) {
//
//}
LaneFSM::LaneFSM(RoadState& rs, Waypoints& wps)
        :rs(rs),
        wps(wps)
{

}

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

bool LaneFSM::ShouldPrepareChange() {
    unordered_map <int, CarPositonData> mInviewCarsFront = rs.mInviewCars;
    if(!mInviewCarsFront.empty()) {
        cout << "cars in front: "<<mInviewCarsFront.size()<<endl;
        return true;
    } else {
        return false;
    }
}

void LaneFSM::LaneKeeping(CarPositonData& egoVehicle, double predictionTimeWindow) {
    unordered_map <int, CarPositonData> mInviewCarsFront = rs.mInviewCars;
    VectorXd egoStartState(6);
    VectorXd egoEndState(6);
    egoStartState << egoVehicle.s, egoVehicle.speed, 0, egoVehicle.d, 0,0;
    if(mInviewCarsFront.size() >0) {
        //todo generate lane changing
    } else {
        //todo lanekeeping
        //todo generate lane keeping path
        egoEndState << (egoVehicle.s + egoVehicle.speed * predictionTimeWindow), egoVehicle.speed, 0, egoVehicle.d, 0,0;
        Trajectory trajectory(egoStartState, egoEndState, 2, 2, 0);

        //these paths are sorted in ascending cost order
        FrenetPath fpath = trajectory.GetSingleFrenetPaths(egoVehicle.s,egoVehicle.speed, 0,  egoVehicle.d,
                                                             0, 0, ConvertLaneToD(ConvertDToLane(egoVehicle.d))).at(0);

//        cout << "fpath_s: "<< vectorToString(fpath.s)<< endl;
//        cout << "fpath_d: "<< vectorToString(fpath.d)<< endl;


        next_x_path.clear();
        next_y_path.clear();

        for(size_t i = 0; i < fpath.s.size(); ++i) {

            vector<double> point_x_y = wps.getXYSimple(fpath.s.at(i), fpath.d.at(i));
            next_x_path.push_back(point_x_y[0]);
            next_y_path.push_back(point_x_y[1]);
        }

        cout << "laneFSM.next_x_path: "<< vectorToString(next_x_path)<< endl;
        cout << "laneFSM.next_y_path: "<< vectorToString(next_y_path)<< endl;
    }
}