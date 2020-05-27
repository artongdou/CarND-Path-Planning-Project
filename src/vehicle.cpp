#include "vehicle.h"

#include "math.h"
#include <iostream>

double mph2mps(double x);

using std::cout;
using std::endl;
using std::string;
using std::vector;

Vehicle::Vehicle() {}

Vehicle::Vehicle(int id)
{
    this->id = id;
}

Vehicle::Vehicle(int id, double x, double y, double yaw, double s, double d, double v)
{
    this->x = x;
    this->y = y;
    this->yaw = yaw;
    this->s = s;
    this->d = d;
    this->v = v;
}

#define LANE_WIDTH 4

int Vehicle::get_lane() {
    return get_lane(this->d);
}

int Vehicle::get_lane(double d)
{
    return floor(d / LANE_WIDTH);
}

Vehicle Vehicle::generate_predictions(double dt)
{
    // Generates predictions for non-ego vehicles to be used in trajectory
    //   generation for the ego vehicle.
    Vehicle pred = *this;
    pred.s = s + v * dt; // Assume constant speed
    pred.d = d; // assume stay in lane
    pred.v = v; // assume constant speed
    return pred;
}

bool Vehicle::get_vehicle_ahead(int lane, vector<Vehicle> &predictions, Vehicle &rVehicle)
{
    vector<Vehicle>::iterator it = predictions.begin();
    bool vehicle_found = false;
    double max_s = 9999;
    while (it != predictions.end()) {
        double dist = it->s - this->s;
        if (lane == it->get_lane(this->d) && dist > 0) {
            vehicle_found = true;
            
            if (fabs(dist) < max_s) {
                rVehicle = *it;
                max_s = dist;
            }
        }
        ++it;
    }
    return vehicle_found;
}

bool Vehicle::get_vehicle_behind(int lane, vector<Vehicle> &predictions, Vehicle &rVehicle)
{
    vector<Vehicle>::iterator it = predictions.begin();
    bool vehicle_found = false;
    double max_s = 9999;
    while (it != predictions.end()) {
        double dist = it->s - this->s;
        if (lane == it->get_lane(this->d) && dist < 0) {
            vehicle_found = true;
            
            if (fabs(dist) < max_s) {
                rVehicle = *it;
                max_s = dist;
            }
        }
        ++it;
    }
    return vehicle_found;
}

Vehicle Vehicle::choose_next_state(vector<Vehicle> &predictions)
{
    // cout << "enter choose next state" << endl;
    vector<string> states = successor_states();
    // float cost;
    // vector<float> costs;
    // vector<vector<Vehicle>> final_trajectories;

    // for (vector<string>::iterator it = states.begin(); it != states.end(); ++it)
    {
        // vector<Vehicle> trajectory = generate_trajectory(*it, predictions);
        // if (trajectory.size() != 0)
        // {
        //     cost = calculate_cost(*this, predictions, trajectory);
        //     costs.push_back(cost);
        //     final_trajectories.push_back(trajectory);
        // }
        return keep_lane_trajectory(predictions);
    }

    // vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
    // int best_idx = distance(begin(costs), best_cost);

    // return final_trajectories[best_idx];
}

Vehicle Vehicle::keep_lane_trajectory(vector<Vehicle> &preditions)
{
    Vehicle vehicle_ahead = Vehicle();
    double dt = 0.02*50;
    Vehicle self_pred = generate_predictions(dt);
    Vehicle trajectory = self_pred;
    if (get_vehicle_ahead(get_lane(), preditions, vehicle_ahead)) {
        cout << "vehicle ahead id: " << vehicle_ahead.id << endl;
        cout << "vehicle ahead speed: " << vehicle_ahead.v << endl;
        cout << vehicle_ahead.s << endl;
        cout << self_pred.s << endl;
        if ((vehicle_ahead.s - self_pred.s) <= 10 ) {
            trajectory.v = vehicle_ahead.v;
        } else {
            trajectory.v = fmin(SPEED_LIMIT, v + dt*9);
        }
    } else {
        trajectory.v = fmin(SPEED_LIMIT, v + dt*9);
    }
    cout << "keep lane speed: " << trajectory.v << endl;
    return trajectory;
}

vector<string> Vehicle::successor_states()
{
    // Provides the possible next states given the current state for the FSM
    //   discussed in the course, with the exception that lane changes happen
    //   instantaneously, so LCL and LCR can only transition back to KL.
    // cout << "enter successor state" << endl;
    vector<string> states;
    states.push_back("KL");
    string state = this->state;
    if (state.compare("KL") == 0)
    {
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