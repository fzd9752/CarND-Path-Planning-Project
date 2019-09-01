#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

#include "constant.h"
#include "planner.h"



// for convenience
using nlohmann::json;
using namespace std;

int main() {
    uWS::Hub h;

    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";
    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;

    std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

    string line;
    while (getline(in_map_, line)) {
        std::istringstream iss(line);
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
        map_waypoints_x.push_back(x);
        map_waypoints_y.push_back(y);
        map_waypoints_s.push_back(s);
        map_waypoints_dx.push_back(d_x);
        map_waypoints_dy.push_back(d_y);
    }

    double ref_vel = 4.0;
    int lane = 1; // 0-left, 1-middle, 2-right
    bool changing = false;
    int waiting = 0;

    h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
                        &map_waypoints_dx,&map_waypoints_dy, &lane, &ref_vel, &changing, &waiting]
                        (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                         uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {

            auto s = hasData(data);

            if (s != "") {
                auto j = json::parse(s);

                string event = j[0].get<string>();

                if (event == "telemetry") {
                    // j[1] is the data JSON object

                    // Main car's localization Data
                    double car_x = j[1]["x"];
                    double car_y = j[1]["y"];
                    double car_s = j[1]["s"];
                    double car_d = j[1]["d"];
                    double car_yaw = j[1]["yaw"];
                    double car_speed = j[1]["speed"];

                    // Previous path data given to the Planner
                    auto previous_path_x = j[1]["previous_path_x"];
                    auto previous_path_y = j[1]["previous_path_y"];
                    // Previous path's end s and d values
                    double end_path_s = j[1]["end_path_s"];
                    double end_path_d = j[1]["end_path_d"];

                    // Sensor Fusion Data, a list of all other cars on the same side
                    //   of the road. [car's unique ID, car's x position in map coordinates,
                    //   car's y position in map coordinates, car's x velocity in m/s,
                    //   car's y velocity in m/s, car's s position in frenet coordinates,
                    //   car's d position in frenet coordinates.
                    auto sensor_fusion = j[1]["sensor_fusion"];

                    json msgJson;

                    vector<double> next_x_vals;
                    vector<double> next_y_vals;

                    /**
                     * TODO: define a path made up of (x,y) points that the car will visit
                     *   sequentially every .02 seconds
                     */

//                    cout << "current_d: " << car_d << endl;
                    if (car_d > (3+4*lane) && car_d < (4*lane+1)) {
                        changing = true;

                    } else {changing = false;}

                    int prev_size = previous_path_x.size();


                    // Prediciton using sensor fusion
                    if (prev_size > 0) {
                        car_s = end_path_s;
                    }

                    bool change_lane = false;

                    vector<vector<double>> vehicles;

                    vector<double> vehicle_dis_ahead = {999.0, 999.0, 999.0};
                    vector<double> vehicle_dis_after = {999.0, 999.0, 999.0};
                    vector<double> vehicle_speed_ahead = {49.5, 49.5, 49.5};


                    for (int i=0; i < sensor_fusion.size(); i++)
                    {
                        vector<double> ve;
                        double x = sensor_fusion[i][1];
                        double y = sensor_fusion[i][2];

                        double vx = sensor_fusion[i][3];
                        double vy = sensor_fusion[i][4];
                        double check_speed = sqrt(vx*vx+vy*vy);
                        double check_car_s = sensor_fusion[i][5];
                        double d = sensor_fusion[i][6];

                        check_car_s += ((double)prev_size * TIME_INTV*check_speed);
                        x += ((double)prev_size * TIME_INTV*vx);
                        y += ((double)prev_size * TIME_INTV*vy);

                        int check_lane;

                        if ((d < 4) && (d >=0)) {
                            check_lane = 0;
                        } else if ((d >= 4) && (d < 8)) {
                            check_lane = 1;
                        } else if ((d >= 8) && (d <= 12)) {
                            check_lane = 2;
                        }

                        ve.push_back(x);
                        ve.push_back(y);
                        ve.push_back(vx);
                        ve.push_back(vy);
                        ve.push_back(check_lane);
                        ve.push_back(check_car_s);

                        vehicles.push_back(ve);

                        double ve_dist = check_car_s-car_s;

                        if (ve_dist > 0) {
                            if (ve_dist < vehicle_dis_ahead[check_lane]) {
                                vehicle_dis_ahead[check_lane] = ve_dist;
                                vehicle_speed_ahead[check_lane] = check_speed;
                            }
                            if ((ve_dist < CHANGE_BUFFER) && (check_lane == lane))  {change_lane = true; }
                        }
                        else {
                            if ((-ve_dist) < vehicle_dis_after[check_lane]) {
                                vehicle_dis_after[check_lane] = -ve_dist;
                            }
                        }

                    }

//                    for (auto dist:vehicle_dis_ahead) {
//                        cout<< "lane dist: " << dist << std::endl;
//                    }
//                    for (auto dist:vehicle_speed_ahead) {
//                        cout<< "lane speed: " << dist << std::endl;
//                    }


//            std::cout << "Find Vehicles: " << vehicles.size() << "\n";

                    vector<double> ptsx;
                    vector<double> ptsy;

                    double ref_x = car_x;
                    double ref_y = car_y;
                    double ref_yaw = deg2rad(car_yaw);

                    if (prev_size < 2) {
                        // make the path tangent to the car
                        double prev_car_x = car_x - cos(car_yaw);
                        double prev_car_y = car_y - sin(car_yaw);

                        ptsx.push_back(prev_car_x);
                        ptsx.push_back(car_x);

                        ptsy.push_back(prev_car_y);
                        ptsy.push_back((car_y));
                    } else {
                        ref_x = previous_path_x[prev_size - 1];
                        ref_y = previous_path_y[prev_size - 1];

                        double ref_x_prev = previous_path_x[prev_size - 2];
                        double ref_y_prev = previous_path_y[prev_size - 2];

                        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

                        ptsx.push_back(ref_x_prev);
                        ptsx.push_back(ref_x);

                        ptsy.push_back(ref_y_prev);
                        ptsy.push_back((ref_y));
                    }

                    for (int i = 0; i < previous_path_x.size(); i++) {
                        next_x_vals.push_back(previous_path_x[i]);
                        next_y_vals.push_back(previous_path_y[i]);
                    }

                    int next_size = 50 - prev_size;

                    vector<double>car_status;
                    car_status.push_back(car_s);
                    car_status.push_back(ref_x);
                    car_status.push_back(ref_y);
                    car_status.push_back(ref_yaw);

                    vector<vector<double>> pre_points;
                    pre_points.push_back(ptsx);
                    pre_points.push_back(ptsy);

                    vector<vector<double>> map_waypoints;
                    map_waypoints.push_back(map_waypoints_s);
                    map_waypoints.push_back(map_waypoints_x);
                    map_waypoints.push_back(map_waypoints_y);

//                    vector<vector<double>> vehicles;
//                    vector<double> vehicle_dis_ahead = {9999.0, 9999.0, 9999.0};
//                    vector<double> vehicle_dis_after = {9999.0, 9999.0, 9999.0};
//                    vector<double> vehicle_speed_ahead = {49.5.0, 49.5.0, 49.5.0};
                    double target_x = 30.0;

                    if (change_lane && !changing && (waiting <= 0)) {
                        cout << "Current Lane: " << lane << "\n";
//                        cout<<"Generate trajectories!"<<endl;
                        vector<int> next_lanes = fms(lane);
                        vector<tk::spline> next_s;
                        vector<vector<double>> pre_waypoints_x;
                        vector<vector<double>> pre_waypoints_y;

                        vector<double> costs;
                        vector<double> target_dist;
                        int ind;
                        vector<double> speeds = {ref_vel, ref_vel, ref_vel};


                        //std::cout<< "Calculate possible trajectories" << "\n";
                        for (std::size_t i = 0; i < next_lanes.size(); i++) {
                            double target_s = 30;
                            double temp_lane = next_lanes[i];
                            double temp_speed = ref_vel;
                            double temp_dist = vehicle_dis_ahead[temp_lane];
                            double goal_speed = vehicle_speed_ahead[temp_lane];


                            vector<double> poss_x;
                            vector<double> poss_y;

                            cout<<"Lane " << temp_lane << "    ";
                            if (temp_dist < SHORT_BUFFER) {temp_speed -= MAX_ACC;}
                            else if (temp_dist < MIDDLE_BUFFER && temp_dist >= SHORT_BUFFER) {
                                if (temp_speed > goal_speed) {
                                    if ((temp_speed - goal_speed) < MAX_ACC) { temp_speed = goal_speed;}
                                    else {temp_speed -= ((temp_speed - goal_speed) / goal_speed) * MAX_ACC;}
                                }
                            } else if (temp_dist < LONG_BUFFER && temp_dist >= MIDDLE_BUFFER) {
                                if (ref_vel < MAX_SPEED) {
                                    temp_speed += (((MAX_SPEED - ref_vel )/ MAX_SPEED) * MAX_ACC);
                                }
//                            cout << "Direct speed: " << temp_speed << endl << "ACC: " << (MAX_SPEED - ref_vel )/ MAX_SPEED << "\n";
//                            cout << "Direct speed: " << temp_speed << endl << "ACC: " << (((MAX_SPEED - ref_vel )/ MAX_SPEED) * MAX_ACC) << "\n";

                            } else {
                                if (ref_vel < MAX_SPEED) { ref_vel += MAX_ACC; }
                            }

                            speeds[temp_lane] = temp_speed;

                            tk::spline poss_s = generate_trajectory(next_lanes[i], target_s, car_status, pre_points, map_waypoints);
//                            cout<<"Get spline!"<<endl;

                            vector<double> goal_xy = getXY(target_s+car_s, 2+4*temp_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                            target_dist.push_back(goal_xy[0]-ref_x);

                            generate_waypoints(poss_s, next_size, temp_speed, goal_xy[0]-ref_x,
                                    car_status, poss_x, poss_y);
//                            cout<<"Get waypoints!"<<endl;

                            pre_waypoints_x.push_back(poss_x);
                            pre_waypoints_x.push_back(poss_x);

                            double closest = 999.0;
                            for (size_t j=0; j<vehicles.size(); j++) {
                                if (vehicles[j][4] == lane || vehicles[j][4] == temp_lane ) {
                                    double dist = nearest_dist(poss_x, poss_y, vehicles[j]);
                                    if (dist < closest) { closest = dist; }
                                }
                            }

                            bool keep_lane = 1;
                            if (temp_lane == lane) { keep_lane = 0;}
                            int middle_lane = 0;
                            if (temp_lane == 1) {middle_lane = -1;}


                            double cost = 10*collision(closest)
                                    + cost_speed(temp_speed)
                                    + cost_space_ahead(temp_lane, next_lanes, vehicle_dis_ahead, AHEAD_BUFFER)
                                    + cost_space_after(temp_lane, next_lanes, vehicle_dis_after, AFTER_BUFFER)
                                    + 0.2*keep_lane;
//                                    + 0.2*middle_lane;

                            cout<<"Cost: "<<cost<< "\n"
                            << " Collision: " << collision(closest) << "\n"
                            << " Speed: " << cost_speed(vehicle_speed_ahead[temp_lane]) << "\n"
                            << " Space ahead: " << cost_space_ahead(temp_lane, next_lanes, vehicle_dis_ahead, AHEAD_BUFFER) << "\n"
                            << " Space after: " << cost_space_after(temp_lane, next_lanes, vehicle_dis_after, AFTER_BUFFER)<< "\n";

                            costs.push_back(cost);
                        }


                        std::vector<double>::iterator min = std::min_element(costs.begin(), costs.end());
                        ind = std::distance(std::begin(costs), min);

                        int lane_id = next_lanes[ind];
                        if (lane == lane_id) {
                            std::cout << "Keep in lane " << next_lanes[ind] << "\n";
                            ref_vel = speeds[lane];

                        } else {
                            std::cout << "Change lane to " << next_lanes[ind] << "\n";
                            lane = next_lanes[ind];
                            ref_vel = speeds[lane];
                            changing = true;
                            waiting = 50;
                        }

                        cout << "---------------------------" << endl;

//                        next_x_vals.insert(next_x_vals.end(), pre_waypoints_x[ind].begin(), pre_waypoints_x[ind].end());
//                        next_y_vals.insert(next_y_vals.end(), pre_waypoints_y[ind].begin(), pre_waypoints_y[ind].end());
                    } else {

                        double temp_speed = ref_vel;
                        double temp_dist = vehicle_dis_ahead[lane];
                        double goal_speed = vehicle_speed_ahead[lane];

//                        cout << temp_dist << endl;
//                        cout << goal_speed << endl;

//                        cout << "DEBUG[INFO]" << "\n";

                        if (temp_dist < SHORT_BUFFER) { temp_speed -= MAX_ACC;
//                        cout << "DEBUG[INFO]1" << "\n";
                        }

                        else if (temp_dist < MIDDLE_BUFFER && temp_dist >= SHORT_BUFFER) {
                            if (temp_speed > goal_speed) {
//                                cout << "DEBUG[INFO2]" << "\n";

                                if ((temp_speed - goal_speed) < MAX_ACC) { temp_speed = goal_speed; }
                                else { temp_speed -= (((temp_speed - goal_speed) / goal_speed) * MAX_ACC); }
                            }
                        } else if (temp_dist < LONG_BUFFER && temp_dist >= MIDDLE_BUFFER) {
                            if (ref_vel < MAX_SPEED) {
                                temp_speed += (((MAX_SPEED - ref_vel )/ MAX_SPEED) * MAX_ACC);
                            }
//                            cout << "Direct speed: " << temp_speed << endl << "ACC: " << (MAX_SPEED - ref_vel )/ MAX_SPEED << "\n";
//                            cout << "Direct speed: " << temp_speed << endl << "ACC: " << (((MAX_SPEED - ref_vel )/ MAX_SPEED) * MAX_ACC) << "\n";

                        } else {
                            if (ref_vel < MAX_SPEED) { temp_speed += MAX_ACC; }
                        }
                        ref_vel = temp_speed;



                    }

                    tk::spline s = generate_trajectory(lane, 30, car_status, pre_points, map_waypoints);
                    vector<double> pre_x;
                    vector<double> pre_y;
                    generate_waypoints(s, next_size, ref_vel, target_x, car_status, pre_x, pre_y);
                    next_x_vals.insert(next_x_vals.end(), pre_x.begin(), pre_x.end());
                    next_y_vals.insert(next_y_vals.end(), pre_y.begin(), pre_y.end());

                    waiting -= 1;

                    /**
                     * End
                     */


                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

                    auto msg = "42[\"control\","+ msgJson.dump()+"]";

                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }  // end "telemetry" if
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }  // end websocket if
    }); // end h.onMessage

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                           char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    } else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }

    h.run();
}