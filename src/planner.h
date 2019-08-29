//
// Created by YishuWang on 29/08/2019.
//
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

#include "constant.h"

#ifndef PATH_PLANNING_PLANNER_H
#define PATH_PLANNING_PLANNER_H

using std::string;
using std::vector;

/*
 * FMS for decide possible lane for the trajectory generator
 * input: int lane number 0-left 1 middle 2-right
 * return: vector of possible lane number
 */
vector<int> fms(int lane) {
    vector<int> lanes;

    lanes.push_back(1);
    if (lane == 1) {
        lanes.push_back(0);
        lanes.push_back(2);
    }
    return lanes;
}

/*
 * Generate polynomial function of the trajectory
 * return a tk::spline
 */
tk::spline generate_trajectory(int lane, vector<double> car_status, vector<vector<double>> pre_points,
                       vector<vector<double>> map_waypoints) {

    double car_s = car_status[0];
    double ref_x = car_status[1];
    double ref_y = car_status[2];
    double ref_yaw = car_status[3];

    vector<double> ptsx = pre_points[0];
    vector<double> ptsy = pre_points[1];

    vector<double> map_waypoints_s = map_waypoints[0];
    vector<double> map_waypoints_x = map_waypoints[1];
    vector<double> map_waypoints_y = map_waypoints[2];



    //add three points ahead current state, space 30m
    vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x,
                                    map_waypoints_y);
    vector<double> next_wp1 = getXY(car_s + 60, (2 + 4 * lane), map_waypoints_s, map_waypoints_x,
                                    map_waypoints_y);
    vector<double> next_wp2 = getXY(car_s + 90, (2 + 4 * lane), map_waypoints_s, map_waypoints_x,
                                    map_waypoints_y);

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);



    // transformation to local car's coordinates
    // car at (0, 0) and yaw 0 degree
    for (int i = 0; i < ptsx.size(); i++) {
        // shift car reference angle to 0 degree
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;

        ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
        ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));

    }

    tk::spline s;

    s.set_points(ptsx, ptsy);


    return s;

}

/*
 * Generate x, y points based on trajectory generator and prediction point
 */
void generate_waypoints(tk::spline s, int next_size, vector<double> car_status, vector<double> &pre_x, vector<double> &pre_y) {

    double ref_x = car_status[1];
    double ref_y = car_status[2];
    double ref_yaw = car_status[3];
    double ref_vel = car_status[4];

    double target_x = TARGET_X;
    double target_y = s(target_x);
    double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));

    double x_add_on = 0;

    // n x 0.02 x velocity = d
    for (int i = 0; i < next_size; i++) {
        double N = (target_dist / (TIME_INTV * ref_vel / MAX_ACC));
        double x_point = x_add_on + (target_x) / N;
        double y_point = s(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

        x_point += ref_x;
        y_point += ref_y;

        pre_x.push_back((x_point));
        pre_y.push_back((y_point));

    }

}

/*
 * Calculate closest distance between generated trajectory and vehicles
 */
double nearest_dist(vector<double> x_points, vector<double> y_points, vector<double> vehicle) {

    double closest = 9999999.0;
    double dist;
    for (int n=0; n<x_points.size(); n++){
        double ve_x = vehicle[0];
        double ve_y = vehicle[1];
        double ve_vx = vehicle[2];
        double ve_vy = vehicle[3];

        ve_x += ve_vx * n * TIME_INTV;
        ve_y += ve_vy * n * TIME_INTV;

        double cur_x = x_points[n];
        double cur_y = y_points[n];

        dist = distance(ve_x, ve_y, cur_x, cur_y);
    }
//    std::cout << "dist: " << dist << "\n";
    if (dist < closest) {
        closest = dist;
    }
//    std::cout << "closest: " << closest << "\n";
    return closest;
}

#endif //PATH_PLANNING_PLANNER_H
