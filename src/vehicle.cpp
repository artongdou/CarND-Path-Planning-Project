#include "vehicle.h"

#include <iostream>

#include "math.h"

double mph2mps(double x);

using std::cout;
using std::distance;
using std::endl;
using std::max_element;
using std::string;
using std::vector;

Vehicle::Vehicle() {}

Vehicle::Vehicle(int id) { this->id = id; }

Vehicle::Vehicle(int id, double x, double y, double yaw, double s, double d,
                 double v) {
  this->x = x;
  this->y = y;
  this->yaw = yaw;
  this->s = s;
  this->d = d;
  this->v = v;
}

#define LANE_WIDTH 4

int Vehicle::get_lane() { return get_lane(this->d); }

int Vehicle::get_lane(double d) { return floor(d / LANE_WIDTH); }

Vehicle Vehicle::generate_predictions(double dt) {
  // Generates predictions for non-ego vehicles to be used in trajectory
  //   generation for the ego vehicle.
  Vehicle pred = *this;
  pred.s = this->s + this->v * dt;  // Assume constant speed
  return pred;
}

bool Vehicle::get_vehicle_ahead(int lane, vector<Vehicle> &predictions,
                                Vehicle &rVehicle) {
  vector<Vehicle>::iterator it = predictions.begin();
  bool vehicle_found = false;
  double max_s = 9999;
  while (it != predictions.end()) {
    double dist = it->s - this->s;
    if (lane == it->get_lane() && dist > 0) {
      vehicle_found = true;

      if (fabs(dist) < max_s) {
        rVehicle = *it;
        max_s = fabs(dist);
      }
    }
    ++it;
  }
  return vehicle_found;
}

bool Vehicle::get_vehicle_behind(int lane, vector<Vehicle> &predictions,
                                 Vehicle &rVehicle) {
  vector<Vehicle>::iterator it = predictions.begin();
  bool vehicle_found = false;
  double max_s = 9999;
  while (it != predictions.end()) {
    double dist = it->s - this->s;
    if (lane == it->get_lane() && dist < 0) {
      vehicle_found = true;

      if (fabs(dist) < max_s) {
        rVehicle = *it;
        max_s = fabs(dist);
      }
    }
    ++it;
  }
  return vehicle_found;
}

vector<Vehicle> Vehicle::choose_next_state(vector<Vehicle> &predictions) {
  // cout << "enter choose next state" << endl;
  vector<string> states = successor_states();
  double cost;
  vector<Vehicle> trajectory;
  vector<double> costs;
  vector<vector<Vehicle>> final_trajectories;

  for (vector<string>::iterator it = states.begin(); it != states.end(); ++it) {
    if (*it == "KL") {
      trajectory = keep_lane_trajectory(predictions);
    } else {
      trajectory = lane_change_trajectory(*it, predictions);
    }
    cost = calculate_cost(trajectory, predictions);
    costs.push_back(cost);
    final_trajectories.push_back(trajectory);
    cout << *it << " cost = " << cost << endl;
    // vector<Vehicle> trajectory = generate_trajectory(*it, predictions);
    // if (trajectory.size() != 0)
    // {
    //     cost = calculate_cost(*this, predictions, trajectory);
    //     costs.push_back(cost);
    //     final_trajectories.push_back(trajectory);
    // }
  }

  // vector<double>::iterator best_cost = min_element(begin(costs), end(costs));
  int best_idx =
      distance(costs.begin(), min_element(costs.begin(), costs.end()));

  return final_trajectories[best_idx];
  // return keep_lane_trajectory(predictions);
}

#define SPEED_WEIGHT 2
#define DIST_WEIGHT 3
#define LANE_CHANGE_WEIGHT 1
double calculate_cost(vector<Vehicle> &trajectory,
                      vector<Vehicle> &predictions) {
  vector<std::function<double(vector<Vehicle> &, vector<Vehicle> &)>> cf_list =
      {speed_cost, safe_distance_cost, lane_change_cost};
  vector<double> weight_list = {SPEED_WEIGHT, DIST_WEIGHT, LANE_CHANGE_WEIGHT};
  double cost = 0;
  for (int i = 0; i < cf_list.size(); ++i) {
    double new_cost = weight_list[i] * cf_list[i](trajectory, predictions);
    cost += new_cost;
  }

  return cost;
}

double speed_cost(vector<Vehicle> &trajectory, vector<Vehicle> &predictions) {
  if (trajectory[1].v > SPEED_LIMIT) {
    return 1;  // max cost
  } else {
    double diff_d = fabs(trajectory[0].d - trajectory[1].d);
    double diff_v = fabs(SPEED_LIMIT - trajectory[1].v);
    return (1 - exp(-diff_v));
  }
}

double lane_change_cost(vector<Vehicle> &trajectory,
                        vector<Vehicle> &predictions) {
  double diff_d = fabs(trajectory[0].d - trajectory[1].d);
  return (1 - exp(-diff_d));
}

double safe_distance_cost(vector<Vehicle> &trajectory,
                          vector<Vehicle> &predictions) {
  double dist_to_veh_ahead = 9999999;
  double dist_to_veh_behind = 9999999;
  Vehicle veh_ahead, veh_behind;
  if (trajectory[1].get_vehicle_ahead(trajectory[1].get_lane(), predictions,
                                      veh_ahead)) {
    dist_to_veh_ahead = veh_ahead.s - trajectory[1].s;
  }
  if (trajectory[1].get_vehicle_behind(trajectory[1].get_lane(), predictions,
                                       veh_behind)) {
    dist_to_veh_behind = -veh_behind.s + trajectory[1].s;
  }
  double min_dist = fmin(fabs(dist_to_veh_ahead), 2 * fabs(dist_to_veh_behind));
  return exp(-fmax(min_dist - 30, 0));
}

vector<Vehicle> Vehicle::keep_lane_trajectory(vector<Vehicle> &predictions) {
  Vehicle vehicle_ahead = Vehicle();
  double dt = 0.02 * 50;
  Vehicle self_pred = generate_predictions(dt);
  vector<Vehicle> trajectory;
  trajectory.push_back(*this);
  if (get_vehicle_ahead(get_lane(), predictions, vehicle_ahead)) {
    cout << "ego vehicle id: " << id << endl;
    cout << "ego vehicle lane: " << get_lane() << endl;
    cout << "vehicle ahead id: " << vehicle_ahead.id << endl;
    cout << "vehicle ahead lane: " << vehicle_ahead.get_lane() << endl;
    cout << "vehicle ahead speed: " << vehicle_ahead.v << endl;
    cout << vehicle_ahead.s << endl;
    cout << self_pred.s << endl;
    if ((vehicle_ahead.s - self_pred.s) <= 20) {
      self_pred.v = vehicle_ahead.v;
    } else {
      self_pred.v = fmin(SPEED_LIMIT, v + dt * 9);
    }
  } else {
    self_pred.v = fmin(SPEED_LIMIT, v + dt * 9);
  }
  cout << "keep lane speed: " << self_pred.v << endl;
  trajectory.push_back(self_pred);
  return trajectory;
}

vector<Vehicle> Vehicle::lane_change_trajectory(string state,
                                                vector<Vehicle> &predictions) {
  int new_lane = get_lane();
  Vehicle vehicle_ahead;
  double dt = 0.02 * 50;
  Vehicle self_pred = generate_predictions(dt);
  vector<Vehicle> trajectory;
  trajectory.push_back(*this);

  if (state == "LCL") {
    new_lane -= 1;
    self_pred.d -= 4;
  } else if (state == "LCR") {
    new_lane += 1;
    self_pred.d += 4;
  }

  if (get_vehicle_ahead(new_lane, predictions, vehicle_ahead)) {
    if ((vehicle_ahead.s - self_pred.s) <= 10) {
      self_pred.v = vehicle_ahead.v;
    } else {
      self_pred.v = fmin(SPEED_LIMIT - 2, v + dt * 9);
    }
  } else {
    self_pred.v = fmin(SPEED_LIMIT - 2, v + dt * 9);
  }
  trajectory.push_back(self_pred);

  return trajectory;
}

vector<string> Vehicle::successor_states() {
  // Provides the possible next states given the current state for the FSM
  //   discussed in the course, with the exception that lane changes happen
  //   instantaneously, so LCL and LCR can only transition back to KL.
  // cout << "enter successor state" << endl;
  vector<string> states;
  states.push_back("KL");
  string state = this->state;
  if (state.compare("KL") == 0) {
    if (get_lane(this->d) != 2) {
      states.push_back("LCR");
    }
    if (get_lane(this->d) != 0) {
      states.push_back("LCL");
    }
  }

  // If state is "LCL" or "LCR", then just return "KL"
  return states;
}