#include <uWS/uWS.h>
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
#include "planner.h"



// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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

    double ref_vel = 5.0;
    int lane = 1; // 0-left, 1-middle, 2-right
    bool changing = false;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane, &ref_vel, &changing]
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



            int prev_size = previous_path_x.size();

            if (changing && ((car_d < 3+4*lane ) && (car_d > 1+4*lane))) {
                changing = false;
            }

            // Prediciton using sensor fusion
            if (prev_size > 0) {
                car_s = end_path_s;
            }

            bool change_lane = false;
            vector<vector<double>> vehicles_ahead;
            vector<vector<double>> vehicles_after;


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

                if (check_car_s > car_s) { vehicles_ahead.push_back(ve); }
                else { vehicles_ahead.push_back(ve); }

                if (check_lane == lane)
                {
                    if ((check_car_s > car_s) && ((check_car_s-car_s) < BUFFER))
                    {
                        std::cout << "Consider to change lane"<< std::endl;
                        change_lane = true;
                    }
                }

            }

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


            //

            int next_size = 50 - previous_path_x.size();

            vector<double>car_status;
            car_status.push_back(car_s);
            car_status.push_back(ref_x);
            car_status.push_back(ref_y);
            car_status.push_back(ref_yaw);
            car_status.push_back(ref_vel);

            vector<vector<double>> pre_points;
            pre_points.push_back(ptsx);
            pre_points.push_back(ptsy);

            vector<vector<double>> map_waypoints;
            map_waypoints.push_back(map_waypoints_s);
            map_waypoints.push_back(map_waypoints_x);
            map_waypoints.push_back(map_waypoints_y);

            if (change_lane)
            {
                ref_vel -= .224;
            }
            else if (ref_vel < MAX_SPEED)
            {
                ref_vel += .224;
            }

//            tk::spline s = generate_trajectory(lane, car_status, pre_points, map_waypoints);
//            vector<double> pre_x;
//            vector<double> pre_y;
//            generate_waypoints(s, next_size, car_status, pre_x, pre_y);





            int target_lane = lane;

            if (change_lane && changing == false) {
                vector<int> next_lanes = fms(lane);
                vector<tk::spline> next_s;
                vector<vector<double>> pre_waypoints_x;
                vector<vector<double>> pre_waypoints_y;
                vector<double> costs;

                vector<double> closests_ahead;
                vector<double> closests_after;
                vector<double> space;


                std::cout<< "Calculate possible trajectories" << "\n";
                for (std::size_t i = 0; i < next_lanes.size(); i++) {
                    tk::spline poss_s = generate_trajectory(next_lanes[i], car_status, pre_points, map_waypoints);
                    vector<double> poss_x;
                    vector<double> poss_y;
                    generate_waypoints(poss_s, next_size, car_status, poss_x, poss_y);
                    pre_waypoints_x.push_back(poss_x);
                    pre_waypoints_x.push_back(poss_x);

                    double dist_ahead;
                    double closest_ahead = 999999.0;
                    double dist_after = 999.0;
                    double closest_after = 999.0;

                    for (std::size_t j=0; j<vehicles_ahead.size(); j++) {
                        if (vehicles_ahead[j][4] == next_lanes[i]) {
                            dist_ahead = nearest_dist(poss_x, poss_y, vehicles_ahead[j]);
//                        std::cout << "dist: " << dist << "\n";
                            if (dist_ahead < closest_ahead) { closest_ahead = dist_ahead; }
                        }

                    }
                    for (std::size_t j=0; j<vehicles_after.size(); j++) {
                        if (vehicles_after[j][4] == next_lanes[i]) {
                            dist_after = nearest_dist(poss_x, poss_y, vehicles_after[j]);
//                        std::cout << "dist: " << dist << "\n";
                            if (dist_after < closest_after) { closest_after = dist_after; }
                        }

                    }
                    std::cout << "For change to lane " << i << "\n";
                    std::cout << "closest: " << closest_ahead << "\n";
                    closests_ahead.push_back(closest_ahead);
                    closests_after.push_back(closest_after);
                    space.push_back(closest_ahead + closest_after);
                    std::cout << "For change to lane " << i << "\n";
                    std::cout << "closest_ahead: " << closest_ahead << "\n";
                    std::cout << "closest_after: " << closest_after << "\n";
                    std::cout << "closest_space: " << closest_ahead + closest_after << "\n";
                }
                std::vector<double>::iterator biggest = std::max_element(closests_ahead.begin(), closests_ahead.end());
                int ind = std::distance(std::begin(closests_ahead), biggest);

                std::cout << "Prepare change lane to " << next_lanes[ind] << "\n";
                if (closests_ahead[ind] >= AHEAD_BUFFER && closests_after[ind] >= AFTER_BUFFER) {
                    target_lane = next_lanes[ind];
                    changing = true;
                    std::cout << "Change Line " <<  changing << "\n";
                }



            }

            lane = target_lane;

            tk::spline s = generate_trajectory(lane, car_status, pre_points, map_waypoints);
            vector<double> pre_x;
            vector<double> pre_y;
            generate_waypoints(s, next_size, car_status, pre_x, pre_y);
//            for (auto it:closests) {
//                std::cout << "closest distance: " << it <<"\n";
//            }

//            generate_waypoints(s, next_size, car_status, pre_x, pre_y);

//            std::cout << "x size: " << pre_x.size() << std::endl;
//            std::cout << "y size: " << pre_y.size() << std::endl;

            next_x_vals.insert(next_x_vals.end(), pre_x.begin(), pre_x.end());
            next_y_vals.insert(next_y_vals.end(), pre_y.begin(), pre_y.end());



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