#include "vehicle.h"

#include <iostream>

using std::cout;
using std::distance;
using std::endl;
using std::max_element;
using std::string;
using std::vector;

/**
 * Constructor
 */
Vehicle::Vehicle() {}

/**
 * Constructor
 * @param id - Vehicle ID
 */
Vehicle::Vehicle(int id) { this->id = id; }

/**
 * Constructor
 * @param id - vehicle id
 * @param x - x coordinate
 * @param y - y coordinate
 * @param yaw - vehcile heading
 * @param s - s in Frenet
 * @param d - d in Frenet
 * @param v - vehicle speed
 */
void Vehicle::init(int id, double x, double y, double yaw, double s, double d,
                   double v) {
  this->x = x;
  this->y = y;
  this->yaw = yaw;
  this->s = s;
  this->d = d;
  this->v = v;
}

/**
 * Function to get the current lane the vehicle is at.
 * @return lane number
 */
int Vehicle::get_lane() { return get_lane(this->d); }

/**
 * Function to get the current lane based on given d in Frenet
 * @param d in Frenet coordinate
 * @return lane number
 */
int Vehicle::get_lane(double d) { return floor(d / LANE_WIDTH); }

/**
 * Function to generate prediction of the vehicle after time dt.
 * It assumes vehicle moves at constant speed.
 * @param dt - time horizon in seconds
 */
Vehicle Vehicle::generate_predictions(double dt) {
  Vehicle pred = *this;
  pred.s = this->s + this->v * dt;  // Assume constant speed
  return pred;
}

/**
 * The functions return whether there is a vehicle ahead in the specified lane
 * @param lane - target lane to check vehicle ahead
 * @param predictions - vector of all predicted vehicles detected in the
 * surroundings
 * @param rVehicle - return the vehicle ahead
 */
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

/**
 * The functions return whether there is a vehicle behind in the specified lane
 * @param lane - target lane to check vehicle behind
 * @param predictions - vector of all predicted vehicles detected in the
 * surroundings
 * @param rVehicle - return the vehicle behind
 */
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

/**
 * Function to generate the possible successor state in finite state machine
 * @return a list of possible successor states
 */
vector<string> Vehicle::successor_states() {
  vector<string> states;
  // states.push_back("KL");
  string state = this->state;
  if (state.compare("KL") == 0) {
    states.push_back("KL");
    if (get_lane() != 2) {
      states.push_back("LCR");
    }
    if (get_lane() != 0) {
      states.push_back("LCL");
    }
  } else if (state.compare("LCL") == 0) {
    if (fabs((2 + 4 * (current_lane - 1)) - d) < 0.2) {
      // Lane change has finished
      states.push_back("KL");
      current_lane -= 1;
    } else {
      states.push_back("LCL");
    }
  } else {
    if (fabs((2 + 4 * (current_lane + 1)) - d) < 0.2) {
      // Lane change has finished
      states.push_back("KL");
      current_lane += 1;
    } else {
      states.push_back("LCR");
    }
  }

  // If state is "LCL" or "LCR", then just return "KL"
  return states;
}

/**
 * The functions return choose the next behavior of the car and return the best
 * trajectory
 * @param predictions - vector of all predicted vehicles detected in the
 * surroundings
 * @return the best trajectory with the lowest cost
 */
vector<Vehicle> Vehicle::choose_next_state(vector<Vehicle> &predictions) {
  // cout << "enter choose next state" << endl;
  vector<string> states = successor_states();
  double cost;
  vector<Vehicle> trajectory;
  vector<double> costs;
  vector<vector<Vehicle>> final_trajectories;

  cout << "-----" << endl;
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
  }

  int best_idx =
      distance(costs.begin(), min_element(costs.begin(), costs.end()));
  cout << "****Best Trajectory*** " << states[best_idx] << endl;
  cout << "current lane = " << current_lane << endl;

  state = states[best_idx];

  return final_trajectories[best_idx];
}

/**
 * Function to generate keep lane trajectory
 * @param predictions - vector of all predicted vehicles detected in the
 * surroundings
 * @return a vector vehicles that represent the keep lane trajectory which
 * consists of the vehicle at {t=0, t=1}
 */
vector<Vehicle> Vehicle::keep_lane_trajectory(vector<Vehicle> &predictions) {
  Vehicle vehicle_ahead = Vehicle();
  double dt = 1.0;
  Vehicle self_pred = generate_predictions(dt);  // predict itself in 1sec
  vector<Vehicle> trajectory;
  trajectory.push_back(*this);
  if (get_vehicle_ahead(get_lane(), predictions, vehicle_ahead)) {
    // cout << "ego vehicle id: " << id << endl;
    // cout << "ego vehicle lane: " << get_lane() << endl;
    // cout << "vehicle ahead id: " << vehicle_ahead.id << endl;
    // cout << "vehicle ahead lane: " << vehicle_ahead.get_lane() << endl;
    // cout << "vehicle ahead speed: " << vehicle_ahead.v << endl;
    // cout << vehicle_ahead.s << endl;
    // cout << self_pred.s << endl;
    if ((vehicle_ahead.s - self_pred.s) <= 12) {
      // slow down to increase buffer from the vehicle ahead
      self_pred.v = vehicle_ahead.v - 2;
    } else if ((vehicle_ahead.s - self_pred.s) <= 20) {
      self_pred.v = vehicle_ahead.v;
    } else {
      self_pred.v = fmin(SPEED_LIMIT, v + dt * 9);
    }
  } else {
    self_pred.v = fmin(SPEED_LIMIT, v + dt * 9);
  }
  // cout << "keep lane speed: " << self_pred.v << endl;
  trajectory.push_back(self_pred);
  return trajectory;
}

/**
 * Function to generate lane change trajectory
 * @param state - "LCR" or "LCL" representing lane change right or left
 * @param predictions - vector of all predicted vehicles detected in the
 * surroundings
 * @return a vector vehicles that represent the keep lane trajectory which
 * consists of the vehicle at {t=0, t=1}
 */
vector<Vehicle> Vehicle::lane_change_trajectory(string state,
                                                vector<Vehicle> &predictions) {
  // int new_lane = get_lane();
  Vehicle vehicle_ahead;
  double dt = 1.0;
  int new_lane;
  Vehicle self_pred = generate_predictions(dt);  // predict itself in 1sec
  vector<Vehicle> trajectory;
  trajectory.push_back(*this);

  if (state == "LCL") {
    new_lane = current_lane - 1;
  } else if (state == "LCR") {
    new_lane = current_lane + 1;
  }

  self_pred.d = 4 * new_lane + 2;

  // if vehicle in front is too close, return keep lane trajectory
  if (get_vehicle_ahead(current_lane, predictions, vehicle_ahead)) {
    if ((vehicle_ahead.s - self_pred.s) <= 10) {
      return keep_lane_trajectory(predictions);
    }
  }

  if (get_vehicle_ahead(new_lane, predictions, vehicle_ahead)) {
    if ((vehicle_ahead.s - self_pred.s) <= 20) {
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