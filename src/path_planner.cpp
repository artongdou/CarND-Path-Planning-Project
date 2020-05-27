#include "path_planner.h"

#include <iostream>

#include "helpers.h"
#include "math.h"
#include "spline.h"
#include "vehicle.h"

using std::cout;
using std::endl;
using std::vector;

double PathPlanner::target_speed = 0;

PathPlanner::PathPlanner() {}

PathPlanner::PathPlanner(vector<double> previous_path_x,
                         vector<double> previous_path_y, double end_path_s,
                         double end_path_d,
                         vector<vector<double>> sensor_fusion,
                         vector<double> map_waypoints_x,
                         vector<double> map_waypoints_y,
                         vector<double> map_waypoints_s) {
  this->previous_path_x = previous_path_x;
  this->previous_path_y = previous_path_y;
  this->end_path_s = end_path_s;
  this->end_path_d = end_path_d;
  this->sensor_fusion = sensor_fusion;
  this->map_waypoints_x = map_waypoints_x;
  this->map_waypoints_y = map_waypoints_y;
  this->map_waypoints_s = map_waypoints_s;

  // go through the list of vehicles detected by sensor fusion
  for (int i = 0; i < sensor_fusion.size(); ++i) {
    Vehicle car = Vehicle();
    vector<double> ith_car = sensor_fusion[i];
    car.id = (int)ith_car[0];
    car.x = ith_car[1];
    car.y = ith_car[2];
    car.v =
        (sqrt(ith_car[3] * ith_car[3] +
              ith_car[4] * ith_car[4]));  // sensor fusion return speed in m/s
    car.s = ith_car[5];
    car.d = ith_car[6];
    cars.push_back(car);
    cout << "id = " << car.id << " d = " << car.d << " lane= " << car.get_lane()
         << " s = " << car.s << " v = " << car.v << endl;
  }
}

void PathPlanner::generate_trajectory(Vehicle& ego,
                                      std::vector<double>& next_x_vals,
                                      std::vector<double>& next_y_vals) {
  vector<Vehicle> predictions;

  vector<Vehicle>::iterator it = this->cars.begin();

  // Gneerate predictions for all the cars on the road
  double dt = 0.02 * 50;  // 1 second
  while (it != this->cars.end()) {
    int v_id = it->id;
    predictions.push_back(it->generate_predictions(dt));
    ++it;
  }

  cout << "ego s= " << ego.s << "ego d= " << ego.d << endl;
  vector<Vehicle> new_trajectory = ego.choose_next_state(predictions);
  cout << "New trajectory target speed: " << new_trajectory[1].v << endl;
  cout << "New trajectory target lane: " << new_trajectory[1].get_lane()
       << endl;

  // Vehicle rVehicle;
  // if (ego.get_vehicle_behind(ego.get_lane(), predictions, rVehicle)) {

  //     cout << "Vehicle Found Ahead" << endl;
  //     cout << "s= " << rVehicle.s << endl;
  // }

  vector<double> anchor_pts_x;
  vector<double> anchor_pts_y;

  target_lane = new_trajectory[1].get_lane();
  // target_speed = new_trajectory.v;

  // if (target_speed < SPEED_LIMIT)
  // {
  //     target_speed += 0.02 * max_accel;
  // }
  // cout << target_speed << endl;
  cout << "target speed: " << target_speed << endl;

  double ref_x;
  double ref_y;
  double ref_yaw;
  double ref_speed;

  // use the previous path as starting points
  int prev_path_size = previous_path_x.size();
  int num_prev_pts = 2;

  if (prev_path_size < num_prev_pts) {
    ref_x = ego.x;
    ref_y = ego.y;
    ref_yaw = deg2rad(ego.yaw);
    anchor_pts_x.push_back(ref_x - cos(ref_yaw));
    anchor_pts_y.push_back(ref_y - sin(ref_yaw));
    anchor_pts_x.push_back(ref_x);
    anchor_pts_y.push_back(ref_y);
    // ref_speed = ego.v;
  } else {
    ref_x = previous_path_x[prev_path_size - 1];
    ref_y = previous_path_y[prev_path_size - 1];
    ref_yaw = deg2rad(ego.yaw);
    for (int i = num_prev_pts; i > 0; --i) {
      anchor_pts_x.push_back(previous_path_x[prev_path_size - i]);
      anchor_pts_y.push_back(previous_path_y[prev_path_size - i]);
    }
    // ref_speed = (previous_path_x[prev_path_size])
  }

  // ego.choose_next_state();

  vector<double> next_anchor0 =
      getXY(ego.s + 60, 2 + target_lane * 4, map_waypoints_s, map_waypoints_x,
            map_waypoints_y);
  vector<double> next_anchor1 =
      getXY(ego.s + 90, 2 + target_lane * 4, map_waypoints_s, map_waypoints_x,
            map_waypoints_y);
  vector<double> next_anchor2 =
      getXY(ego.s + 120, 2 + target_lane * 4, map_waypoints_s, map_waypoints_x,
            map_waypoints_y);

  anchor_pts_x.push_back(next_anchor0[0]);
  anchor_pts_x.push_back(next_anchor1[0]);
  anchor_pts_x.push_back(next_anchor2[0]);
  anchor_pts_y.push_back(next_anchor0[1]);
  anchor_pts_y.push_back(next_anchor1[1]);
  anchor_pts_y.push_back(next_anchor2[1]);

  for (int i = 0; i < anchor_pts_x.size(); ++i) {
    // shift to car local reference frame
    double shift_x = anchor_pts_x[i] - ref_x;
    double shift_y = anchor_pts_y[i] - ref_y;
    anchor_pts_x[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
    anchor_pts_y[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
  }

  tk::spline s;
  s.set_points(anchor_pts_x, anchor_pts_y);

  // start with all the previous points
  for (int i = 0; i < prev_path_size; ++i) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  // add new points
  double target_x = 30;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x * target_x + target_y * target_y);
  double x_add_on = 0;

  for (int i = 0; i < 50 - prev_path_size; ++i) {
    if (new_trajectory[1].v > target_speed) {
      target_speed = fmin(target_speed + MAX_JERK * 0.02, new_trajectory[1].v);
    } else if (new_trajectory[1].v < target_speed) {
      target_speed = fmax(target_speed - MAX_JERK * 0.02, new_trajectory[1].v);
    }

    double N = target_dist / (0.02 * target_speed);
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
}
