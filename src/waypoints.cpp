//
// Created by Wing Yee mak on 18/06/2018.
//
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iostream>
#include "waypoints.h"


Waypoints::Waypoints() {

    bool isFirst= true;
    double x0, y0, dx0, dy0;

    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";
    // The max s value before wrapping around the track back to 0


    ifstream in_map_(map_file_.c_str(), ifstream::in);

    string line;
    while (getline(in_map_, line)) {
        istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        waypoints_x.push_back(x);
        waypoints_y.push_back(y);
        waypoints_s.push_back(s);
        waypoints_dx.push_back(d_x);
        waypoints_dy.push_back(d_y);

        if (isFirst) {
            x0 = x;
            y0 = y;
            dx0 = d_x;
            dy0 = d_y;

            isFirst = false;
        }
    }

    waypoints_x.push_back(x0);
    waypoints_y.push_back(y0);
    waypoints_dx.push_back(dx0);
    waypoints_dy.push_back(dy0);
    waypoints_s.push_back(max_s);


    calcSplineTrack();
}

void Waypoints::calcSplineTrack() {

    cout << "calc split track start"<< endl;
    cout << "waypoints_s: "<< waypoints_s.size()<<endl;
    cout << "waypoints_x"<< waypoints_x.size()<<endl;
    cout << "waypoints_y"<< waypoints_y.size()<<endl;

    xspline.set_points(waypoints_s, waypoints_x);
    yspline.set_points(waypoints_s, waypoints_y);
    dx_spline.set_points(waypoints_s, waypoints_dx);
    dy_spline.set_points(waypoints_s, waypoints_dy);

    cout << "calc split track end"<< endl;

    waypoints_x.pop_back();
    waypoints_y.pop_back();
    waypoints_s.pop_back();
    waypoints_dx.pop_back();
    waypoints_dy.pop_back();
    waypoints_normx.pop_back();
    waypoints_normy.pop_back();

    for (double s = 0; s <= floor(max_s); s++) {
        double x = xspline(s);
        double y = yspline(s);
        double dx = dx_spline(s);
        double dy = dy_spline(s);

        hp_waypoints_x.push_back(x);
        hp_waypoints_y.push_back(y);
        hp_waypoints_dx.push_back(dx);
        hp_waypoints_dy.push_back(dy);
    }

    for (int i = 0; i< hp_waypoints_x.size(); i++) {
        hp_waypoints_s.push_back(i);
    }


}

vector<double> Waypoints::getFrenet(double x, double y, double theta) {
    vector<double> maps_x = this->hp_waypoints_x;
    vector<double> maps_y = this->hp_waypoints_y;
    int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

    int prev_wp;
    prev_wp = next_wp-1;
    if(next_wp == 0)
    {
        prev_wp  = (int)maps_x.size()-1;
    }

    double n_x = maps_x[next_wp]-maps_x[prev_wp];
    double n_y = maps_y[next_wp]-maps_y[prev_wp];
    double x_x = x - maps_x[prev_wp];
    double x_y = y - maps_y[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
    double proj_x = proj_norm*n_x;
    double proj_y = proj_norm*n_y;

    double frenet_d = distance(x_x,x_y,proj_x,proj_y);

    //see if d value is positive or negative by comparing it to a center point

    double center_x = 1000-maps_x[prev_wp];
    double center_y = 2000-maps_y[prev_wp];
    double centerToPos = distance(center_x,center_y,x_x,x_y);
    double centerToRef = distance(center_x,center_y,proj_x,proj_y);

    if(centerToPos <= centerToRef)
    {
        frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = 0;
    for(int i = 0; i < prev_wp; i++)
    {
        frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
    }

    frenet_s += distance(0,0,proj_x,proj_y);

    return {frenet_s,frenet_d};

}

vector<double> Waypoints::getXY(double s, double d) {
    if (s > max_track_s) {
        s = s - max_track_s;
    }

    int prev_wp = -1;

    while(s > hp_waypoints_s[prev_wp+1] && (prev_wp < (int)(hp_waypoints_s.size()-1) ))
    {
        prev_wp++;
    }

    int wp2 = (prev_wp+1)%(int)hp_waypoints_x.size();

    double heading = atan2((hp_waypoints_y[wp2]-hp_waypoints_y[prev_wp]),(hp_waypoints_s[wp2]-hp_waypoints_s[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s- hp_waypoints_s[prev_wp]);

    double seg_x = hp_waypoints_x[prev_wp]+seg_s*cos(heading);
    double seg_y = hp_waypoints_y[prev_wp]+seg_s*sin(heading);

    double perp_heading = heading-pi()/2;

    double x = seg_x + d*cos(perp_heading);
    double y = seg_y + d*sin(perp_heading);

    return {x,y};
}

double Waypoints::xySpeedToFrenetSpeed(double Vxy, double s) {
    s = fmod(s, max_track_s);
    double dx_ds = xspline.deriv(1, s);
    double dy_ds = yspline.deriv(1, s);
    double Vs = (Vxy / sqrt(dx_ds * dx_ds + dy_ds * dy_ds));
    return Vs;
}

//int Waypoints::ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
//{
//
//    double closestLen = 100000; //large number
//    int closestWaypoint = 0;
//
//    for(int i = 0; i < maps_x.size(); i++)
//    {
//        double map_x = maps_x[i];
//        double map_y = maps_y[i];
//        double dist = distance(x,y,map_x,map_y);
//        if(dist < closestLen)
//        {
//            closestLen = dist;
//            closestWaypoint = i;
//        }
//
//    }
//
//    return closestWaypoint;
//
//}

//WaypointData Waypoints::ClosestWaypoint(double x, double y)
//{
//
//    double closestLen = 100000; //large number
//    WaypointData closestWaypoint;
//
//    for(int i = 0; i < waypointsList.size(); i++)
//    {
//        WaypointData wp = waypointsList.at(i);
//        double dist = distance(x,y,wp.x,wp.y);
//        if(dist < closestLen)
//        {
//            closestLen = dist;
//            closestWaypoint = wp;
//        }
//
//    }
//
//    return closestWaypoint;
//
//}


