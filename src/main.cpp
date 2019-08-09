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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
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
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          // Define the actual (x, y) points for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          // start in lane one
          int lane = 1;

          // Have a reference velocity to target
          double ref_vel = 49.5;   //mph (the speed limit is 50 mph)

          int prev_size = previous_path_x.size();

          // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m.
          // Will use these waypoints with spline
          vector<double> ptsx;
          vector<double> ptsy;

          // reference x, y, yaw state
          // We will reference the starting point as where the car is or at the previous path end point.
          double ref_x = car_x;
          double ref_y = car_y;
          duoble ref_yaw = deg2rad(car_yaw);

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
            ref_x = previous_path_x[-1];
            ref_y = previous_path_y[-1];

            double prev_ref_x = previous_path_x[-2];
            double prev_ref_y = previous_path_y[-2];
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

          // shift car reference angle to 0 degree (all the weightpoints on x axis)
          for (int i = 0; i < ptsx.size(); i++){
            double shift_x = ptsx[i] - ref_x; // use (ref_x, ref_y) as origin (0,0)
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
            ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
          }

          // create a spline
          tk::spline s;

          // Add those 5 anchor points(waypoints) to the spline
          s.set_points(ptsx, ptsy);

          // Start with all of the previous path points form last time
          // Every time we will create some waypoints(say 50), but the simulator will only take a few points(say 5),
          // So, the previous_path will have 45 waypoints left, and we'll start with those 45 waypoints.
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

          /*  car can move in the lane but not smooth (violate acceleration and jerk)
          double dist_inc = 0.45;
          for (int i = 0; i < 50; i++){
            double next_s = car_s + (i+1) * dist_inc;
            double next_d = 6;
            vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            next_x_vals.push_back(xy[0]);
            next_y_vals.push_back(xy[1]);
          }
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
