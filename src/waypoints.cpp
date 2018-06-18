//
// Created by Wing Yee mak on 18/06/2018.
//
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iostream>
#include "waypoints.h"


Waypoints::Waypoints() {
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

    // Waypoint map to read from
    const string map_file_ = "../data/highway_map.csv";

    ifstream in_map_(map_file_.c_str(), ifstream::in);

    string line;
    while (getline(in_map_, line)) {
        istringstream iss(line);
        WaypointData wp;
        iss >> wp.x;
        iss >> wp.y;
        iss >> wp.s;
        iss >> wp.d_x;
        iss >> wp.d_y;
        waypointsList.push_back(wp);
    }
}
