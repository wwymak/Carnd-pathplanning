#include "EgoVehicle.h"
#include "vector"

using namespace std;

void EgoVehicle::updatePath(CarPositonData pos , vector<double> previous_path_x, vector<double> previous_path_y, const vector<double> maps_s, const vector<double> maps_x, const vector<double> maps_y) {
//    vector<FrenetPath> = trajectory.GetFrenetPaths(double current_s,double current_s_dot, double current_s_dotdot,  double current_d,
//            double current_d_dot, double current_d_ddotdot,double  target_d)

    next_path_x.clear();
    next_path_y.clear();

    const int prev_path_size = (int)previous_path_x.size();

    if(prev_path_size > 2) {
        next_path_x.push_back(previous_path_x.at(prev_path_size -2));
        next_path_x.push_back(previous_path_x.at(prev_path_size -1));
        next_path_y.push_back(previous_path_x.at(prev_path_size -2));
        next_path_y.push_back(previous_path_x.at(prev_path_size -1));
    }

//    cout << "ego vehicle update path"<< endl;
//    cout << "car positiion data"<< pos.s << "," << pos.d << endl;
    //todo fix, don't htink it's quite correct
    //candidate paths => use prev path?
    vector<FrenetPath> candidatePaths =  trajectory.GetFrenetPaths(pos.s, pos.speed, 0.0, pos.d, 0, 0, 6);
    cout << "candidate paths" << candidatePaths.size()<< endl;
    vector<FrenetPath> validPaths = trajectory.GetValidPaths(candidatePaths);
    cout<< "valid paths calculated"<< validPaths.size()<< endl;
    FrenetPath bestPath = trajectory.GetOptimalPath(validPaths);
    cout<< "bestPath calculated"<< bestPath.s.size() << endl;
    for (int i = 0; i< bestPath.s.size(); i++) {
        vector<double> xy = getXY(bestPath.s.at(i), bestPath.d.at(i), maps_s, maps_x, maps_y );
        next_path_x.push_back(xy.at(0));
        next_path_y.push_back(xy.at(1));
    }
};
