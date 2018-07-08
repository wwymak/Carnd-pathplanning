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

    calcSplineTrack();
}

void Waypoints::calcSplineTrack() {
    vector<double> x, y, s;
    x.resize(waypointsList.size());
    y.resize(waypointsList.size());
    s.resize(waypointsList.size());
    for (size_t i = 0; i < waypointsList.size(); i++) {
        const auto& w = waypointsList.at(i);
        x[i] = w.x;
        y[i] = w.y;
        s[i] = w.s;
    }

    xspline.set_points(s, x);
    yspline.set_points(s, y);
}

vector<double> Waypoints::getXY(double s, double d) {
    if (s > max_track_s) {
        s = s - max_track_s;
    }

    double xpoint = xspline(s);
    double ypoint = yspline(s);
    // normal vector
//    Point nv(-m_y_spline.deriv(1,s), m_x_spline.deriv(1,s));
//    return pt + nv * d;

    return {xpoint, ypoint};
}

vector<double> Waypoints::getXYSimple(double s, double d)
{
    int prev_wp = -1;

    while(s > waypointsList[prev_wp+1].s && (prev_wp < (int)(waypointsList.size()-1) ))
    {
        prev_wp++;
    }

    int wp2 = (prev_wp+1)%waypointsList.size();

    double heading = atan2((waypointsList[wp2].y- waypointsList[prev_wp].y),(waypointsList[wp2].x - waypointsList[prev_wp].x));
    // the x,y,s along the segment
    double seg_s = (s - waypointsList[prev_wp].s);

    double seg_x = waypointsList[prev_wp].x+seg_s*cos(heading);
    double seg_y = waypointsList[prev_wp].y + seg_s*sin(heading);

    double perp_heading = heading-pi()/2;

    double x = seg_x + d*cos(perp_heading);
    double y = seg_y + d*sin(perp_heading);

    return {x,y};

}
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
//
//WaypointData Waypoints::NextWaypoint(double x, double y)
//{
//
//    WaypointData closestWp = ClosestWaypoint(x,y);
//
//
//    double heading = atan2((closestWp.y-y),(closestWp.x-x));
//
//    double angle = fabs(theta-heading);
//    angle = min(2*pi() - angle, angle);
//
//    if(angle > pi()/4)
//    {
//        closestWaypoint++;
//        if (closestWaypoint == maps_x.size())
//        {
//            closestWaypoint = 0;
//        }
//    }
//
//    return closestWaypoint;
//}
//
//// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
//vector<double> Waypoints::getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
//{
//    int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);
//
//    int prev_wp;
//    prev_wp = next_wp-1;
//    if(next_wp == 0)
//    {
//        prev_wp  = maps_x.size()-1;
//    }
//
//    double n_x = maps_x[next_wp]-maps_x[prev_wp];
//    double n_y = maps_y[next_wp]-maps_y[prev_wp];
//    double x_x = x - maps_x[prev_wp];
//    double x_y = y - maps_y[prev_wp];
//
//    // find the projection of x onto n
//    double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
//    double proj_x = proj_norm*n_x;
//    double proj_y = proj_norm*n_y;
//
//    double frenet_d = distance(x_x,x_y,proj_x,proj_y);
//
//    //see if d value is positive or negative by comparing it to a center point
//
//    double center_x = 1000-maps_x[prev_wp];
//    double center_y = 2000-maps_y[prev_wp];
//    double centerToPos = distance(center_x,center_y,x_x,x_y);
//    double centerToRef = distance(center_x,center_y,proj_x,proj_y);
//
//    if(centerToPos <= centerToRef)
//    {
//        frenet_d *= -1;
//    }
//
//    // calculate s value
//    double frenet_s = 0;
//    for(int i = 0; i < prev_wp; i++)
//    {
//        frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
//    }
//
//    frenet_s += distance(0,0,proj_x,proj_y);
//
//    return {frenet_s,frenet_d};
//
//}
//
//// Transform from Frenet s,d coordinates to Cartesian x,y
//vector<double> Waypoints::getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
//{
//    int prev_wp = -1;
//
//    while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
//    {
//        prev_wp++;
//    }
//
//    int wp2 = (prev_wp+1)%maps_x.size();
//
//    double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
//    // the x,y,s along the segment
//    double seg_s = (s-maps_s[prev_wp]);
//
//    double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
//    double seg_y = maps_y[prev_wp]+seg_s*sin(heading);
//
//    double perp_heading = heading-pi()/2;
//
//    double x = seg_x + d*cos(perp_heading);
//    double y = seg_y + d*sin(perp_heading);
//
//    return {x,y};
//
//}
