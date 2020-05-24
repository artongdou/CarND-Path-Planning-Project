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
using std::cout;
using std::endl;
using std::string;
using std::vector;

int main()
{
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
  while (getline(in_map_, line))
  {
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

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                                                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {

      auto s = hasData(data);

      if (s != "")
      {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry")
        {
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

          vector<double> next_x_vals;
          vector<double> next_y_vals;
          vector<double> anchor_pts_x;
          vector<double> anchor_pts_y;

          double ref_speed = 49.5; // 49.5mph
          int lane = 1;

          double ref_x;
          double ref_y;
          double ref_yaw;

          // use the previous path as starting points
          int prev_path_size = previous_path_x.size();
          int num_prev_pts = 2;

          if (prev_path_size < num_prev_pts)
          {
            ref_x = car_x;
            ref_y = car_y;
            ref_yaw = deg2rad(car_yaw);
            anchor_pts_x.push_back(ref_x - cos(ref_yaw));
            anchor_pts_y.push_back(ref_y - sin(ref_yaw));
            anchor_pts_x.push_back(ref_x);
            anchor_pts_y.push_back(ref_y);
          }
          else
          {
            ref_x = previous_path_x[prev_path_size - 1];
            ref_y = previous_path_y[prev_path_size - 1];
            ref_yaw = deg2rad(car_yaw);
            for (int i = num_prev_pts; i > 0; --i)
            {
              anchor_pts_x.push_back(previous_path_x[prev_path_size - i]);
              anchor_pts_y.push_back(previous_path_y[prev_path_size - i]);
            }
          }

          vector<double> next_anchor0 = getXY(car_s + 30, 2 + lane * 4, map_waypoints_s,
                                              map_waypoints_x,
                                              map_waypoints_y);
          vector<double> next_anchor1 = getXY(car_s + 60, 2 + lane * 4, map_waypoints_s,
                                              map_waypoints_x,
                                              map_waypoints_y);
          vector<double> next_anchor2 = getXY(car_s + 90, 2 + lane * 4, map_waypoints_s,
                                              map_waypoints_x,
                                              map_waypoints_y);

          anchor_pts_x.push_back(next_anchor0[0]);
          anchor_pts_x.push_back(next_anchor1[0]);
          anchor_pts_x.push_back(next_anchor2[0]);
          anchor_pts_y.push_back(next_anchor0[1]);
          anchor_pts_y.push_back(next_anchor1[1]);
          anchor_pts_y.push_back(next_anchor2[1]);

          for (int i = 0; i < anchor_pts_x.size(); ++i)
          {
            // shift to car local reference frame
            double shift_x = anchor_pts_x[i] - ref_x;
            double shift_y = anchor_pts_y[i] - ref_y;
            anchor_pts_x[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
            anchor_pts_y[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
          }

          tk::spline s;
          s.set_points(anchor_pts_x, anchor_pts_y);

          // start with all the previous points
          for (int i = 0; i < prev_path_size; ++i)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // add new points
          double target_x = 30;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x * target_x + target_y * target_y);
          double x_add_on = 0;
          for (int i = 0; i < 50 - prev_path_size; ++i)
          {
            double N = target_dist / (0.02 * ref_speed * 0.45);
            // cout << "N= " << N << endl;
            double x_point = x_add_on + target_x / N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          // next_x_vals.clear();
          // next_y_vals.clear();
          // double dist_inc = 0.5;
          // for (int i = 0; i < 50; ++i)
          // {
          //   vector<double> next = getXY(car_s+dist_inc*(i+1), car_d, map_waypoints_s, 
          //            map_waypoints_x, 
          //            map_waypoints_y);
          //   next_x_vals.push_back(next[0]);
          //   next_y_vals.push_back(next[1]);
          // }

          cout << "next_x_vals size: " << next_x_vals.size() << endl;
          cout << "next_y_vals size: " << next_y_vals.size() << endl;
          cout << next_x_vals[0] << endl;
          cout << next_y_vals[0] << endl;
          cout << next_x_vals[1] << endl;
          cout << next_y_vals[1] << endl;
          cout << next_x_vals[2] << endl;
          cout << next_y_vals[2] << endl;
          cout << next_x_vals[3] << endl;
          cout << next_y_vals[3] << endl;
          cout << next_x_vals[4] << endl;
          cout << next_y_vals[4] << endl;
          cout << next_x_vals[5] << endl;
          cout << next_y_vals[5] << endl;
          cout << next_x_vals[6] << endl;
          cout << next_y_vals[6] << endl;
          cout << next_x_vals[49] << endl;
          cout << next_y_vals[49] << endl;

          cout << prev_path_size << endl;
          cout << "------" << endl;

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        } // end "telemetry" if
      }
      else
      {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    } // end websocket if
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
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}