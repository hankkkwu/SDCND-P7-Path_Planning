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
  // Have a reference velocity to target
  double ref_vel = 0.0;   //mph (the speed limit is 50 mph)

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

  h.onMessage([&ref_vel,&map_waypoints_x,&map_waypoints_y,
               &map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy]
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

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          // The data format for each car is: [ id, x, y, vx, vy, s, d]. The id is a unique identifier for that car.
          // The x, y values are in global map coordinates, and the vx, vy values are the velocity components,
          // also in reference to the global map. Finally s and d are the Frenet coordinates for that car.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          // Define the actual (x, y) points for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

           // the lane of our car
           int lane = (int)car_d / 4;

          int prev_size = previous_path_x.size();

          if (prev_size > 0){
            car_s = end_path_s;   //our car's previous path's end s values
          }

          bool too_close = false;               // check if front car is too close
          bool left_lane_available = true;      // check if left lane is safe to go
          bool right_lane_available = true;     // check if right lane is safe to go
          bool most_right_lane = true;          // check if lane 2 within some range has any car or not, this is for the situation
                                                // that our car in lane 1, and try to change lanes
          bool most_left_lane = true;           // check if lane 0 within some range has any car or not
          double front_car_speed;               // speed of the car in front of us
          double front_car_position;            // s value of the car in front of us

          // using sensor fusion data to avoid collision
          for (int i = 0; i < sensor_fusion.size(); i++){
            float d = sensor_fusion[i][6];
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx+vy*vy);
            double check_car_s = sensor_fusion[i][5];   // current s

            // check if the other cars in my lane
            if (d < (2+4*lane+2) && d > (2+4*lane-2)){
              // predect the car's s in the future
              check_car_s += (double)prev_size * 0.02 * check_speed;

              if ((car_s < check_car_s) && (check_car_s - car_s) < 30){
                // if the other car is in front of us and the gap between the other car and our car is smaller than 30 meters,
                // then we need to take action (lower our speed or change lanes).
                too_close = true;
                front_car_speed = check_speed;
                front_car_position = check_car_s;
                if (lane == 0 || lane == 2){
                  most_left_lane = false;
                  most_right_lane = false;
                }
              }
            }
            // if the other car not in my lane, see which lane it is.
            else{
              int other_car_lane  = (int)d / 4;   // the lane of other car

              // predect the car's s in the future
              check_car_s += (double)prev_size * 0.02 * check_speed;

              // check if there is a car in adjecent lane within the range (-10m ~ 80m)
              if ((check_car_s - car_s) < 80 && (check_car_s - car_s) > -10){

                if ((lane > other_car_lane) && abs(lane-other_car_lane) == 1 && (check_car_s - car_s) < 30 && (check_car_s - car_s) > -10){
                  // if our car in lane 2 and the other car in lane 1, or our car in lane 1 and the other car in lane 0,
                  // check if there are cars with in this range (-10m ~ 30m) in the left lane
                  left_lane_available = false;
                }
                else if ((lane < other_car_lane) && abs(lane-other_car_lane) == 1 && (check_car_s - car_s) < 30 && (check_car_s - car_s) > -10){
                  // if our car in lane 1 and the other car in lane 2, or our car in lane 0 and the other car in lane 1
                  // check if there are cars with in this range (-10m ~ 30m) in the right lane
                  right_lane_available = false;
                }
                if (other_car_lane == 0){
                  // if there is other car in lane 0, then change to lane 0 might not be a good choice
                  most_left_lane = false;
                }
                else if (other_car_lane == 2){
                  // if there is other car in lane 2, then change to lane 2 might not be a good choice
                  most_right_lane = false;
                }
              }
            }
          }

          for (int i = 0; i < sensor_fusion.size(); i++){
            float d = sensor_fusion[i][6];
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx+vy*vy);
            double check_car_s = sensor_fusion[i][5];   // current s

            // check if the other cars in my lane
            if (d < (2+4*lane+2) && d > (2+4*lane-2)){
              continue;
            }
            else{
              int other_car_lane  = (int)d / 4;
              // predect the car's s in the future
              check_car_s += (double)prev_size * 0.02 * check_speed;

              double adjecent_car_pos = check_car_s;
              // Make behavior planning more solid when changing lanes
              if (fabs(adjecent_car_pos - front_car_position) < 10){
                if (lane == 1 && other_car_lane == 2){
                  right_lane_available = false;
                }
                else if (lane == 1 && other_car_lane == 0){
                  left_lane_available = false;
                }
                else if (lane == 0 && other_car_lane == 1){
                  right_lane_available = false;
                }
                else if (lane == 2 && other_car_lane == 1){
                  left_lane_available = false;
                }
              }
            }
          }

          if (too_close){
            // if our car is too close to the front car, then consider change lanes
            if ((left_lane_available) && (lane > 0)){
              // if left lane is safe to change and our car is not in lane 0, we can consider change to left lane
              if (most_right_lane && !most_left_lane){
                // if our car in lane 1, and there is no car in lane 2 within 90 meters,
                // but there is car in lane 0 within 120 meters, then change to right lane
                lane += 1;
              }
              else{
                // if there is car in lane 2 within 120 meters then change to left lane
                lane -= 1;
              }
              if(ref_vel < 49.5){
                ref_vel += 0.1;
              }
            }
            else if ((right_lane_available) && (lane < 2)){
              // if right lane is safe to change and our car is not in lane 2, we can consider change to right lane
              if (most_left_lane && !most_right_lane){
                // if our car in lane 1, and there is no car in lane 0 within 90 meters,
                // but there is car in lane 2 within 120 meters, then change to left lane
                lane -= 1;
              }
              else{
                // if there is car in lane 0 within 120 meters then change to right lane
                lane += 1;
              }
              if(ref_vel < 49.5){
                ref_vel += 0.1;
              }
            }
            else{
              // if lane change is not available, then try to slow down to front car's speed
              if (ref_vel > front_car_speed){
                ref_vel -= (ref_vel - front_car_speed)/2.24 * 0.01;
              }
              else{
                ref_vel = front_car_speed;
              }
            }
          }

          else if (ref_vel < 49.5){
            if (ref_vel < 30){
              ref_vel += 0.4;
            }
            else{
              ref_vel += 0.25;
            }
          }

          // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m.
          // Will use these waypoints with spline
          vector<double> ptsx;
          vector<double> ptsy;

          // reference x, y, yaw state
          // We will reference the starting point as where the car is or at the previous path end point.
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // If the previous path size smaller than 2, use where the car is as the starting point
          if (prev_size < 2){
            // Use 2 points that make the path tangent to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          // Use the previous path's end point as starting point
          else{
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double prev_ref_x = previous_path_x[prev_size-2];
            double prev_ref_y = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-prev_ref_y, ref_x-prev_ref_x);

            ptsx.push_back(prev_ref_x);
            ptsx.push_back(ref_x);
            ptsy.push_back(prev_ref_y);
            ptsy.push_back(ref_y);
          }

          // first argument : add evenly 30m spaced points ahead of the starting reference point
          // second argument : the lane is 4 meter wide, center of most left lane is 2, center of the middle lane is 6, center of most right lane is 10
          vector<double> next_wp0 = getXY(car_s+30, (2+lane*4), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60, (2+lane*4), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90, (2+lane*4), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          for (int i = 0; i < ptsx.size(); i++){
            // use (ref_x, ref_y) as origin(0,0), and shift all the waypoints
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            // rotate car reference angle to 0 degree
            ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
            ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
          }

          // create a spline
          tk::spline s;

          // Add those 5 anchor points(waypoints) to the spline
          s.set_points(ptsx, ptsy);

          // Start with all of the previous path points from last time
          // Every time we will create some waypoints(say 50), but the simulator will only take a few points(say 5),
          // therefore, the previous_path will have 45 waypoints left, and we'll start with those 45 waypoints.
          for (int i = 0; i < previous_path_x.size(); i++){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Calculate how to break up spline points so that we travel at our desired reference velocity
          double target_x = 30.0;   // set horizon value = 30
          double target_y = s(target_x);   // giving the spline x value to get y
          double target_dist = sqrt(target_x*target_x + target_y*target_y);   // the distance from our car to the target point

          // Since we shifted our 5 anchor points(waypoints) to start at the origin(0,0), so we x will start at 0
          double x_add_on = 0;

          // Fill up the rest of our path planner after filling with previous points:
          // Set waypoints = 50
          for (int i = 0; i < 50-previous_path_x.size(); i++){
            double N = target_dist / (0.02 * ref_vel / 2.24);   //ref_vel is mph, devide by 2.24 to become m/s
            double x_point = x_add_on + target_x / N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_reference = x_point;
            double y_reference = y_point;

            // Rotate back to normal
            x_point = x_reference * cos(ref_yaw) - y_reference * sin(ref_yaw);
            y_point = x_reference * sin(ref_yaw) + y_reference * cos(ref_yaw);

            // shift back to original
            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

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
