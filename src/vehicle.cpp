#include "vehicle.h"

#include "math.h"

using std::string;
using std::vector;

Vehicle::Vehicle() {}

Vehicle::Vehicle(int id)
{
    this->id = id;
}

Vehicle::Vehicle(int id, double x, double y, double yaw, double s, double d, double vx, double vy)
{
    this->x = x;
    this->y = y;
    this->yaw = yaw;
    this->s = s;
    this->d = d;
    this->vx = vx;
    this->vy = vy;
}

#define LANE_WIDTH 4
int Vehicle::get_lane(double d)
{
    return floor(d / LANE_WIDTH);
}

Vehicle Vehicle::generate_predictions(double dt)
{
    // Generates predictions for non-ego vehicles to be used in trajectory
    //   generation for the ego vehicle.
    Vehicle pred = Vehicle();
    double speed = sqrt(vx*vx + vy*vy);
    pred.s = s + speed * dt; // Assume constant speed
    pred.d = d; // assume stay in lane
    pred.vx = vx; // assume constant speed
    pred.vy = vy; // assume constant speed
    return pred;
}

bool Vehicle::get_vehicle_ahead(int lane, vector<Vehicle> &predictions, Vehicle &rVehicle)
{
    Vehicle::iterator it = predictions.begin();
    Vehicle temp_vehicle;
    bool vehicle_found = false;
    double max_s = 9999;
    while (it != predictions.end()) {
        double dist = it->s - this->s;
        if (lane == it->get_lane(this->d) && dist < 0) {
            vehicle_found = true;
            
            if (fabs(dist) < max_s) {
                temp_vehicle = *it;
                max_s = dist;
            }
        }
        ++it;
    }
    return vehicle_found;
}

bool Vehicle::get_vehicle_behind(int lane, vector<Vehicle> &predictions, Vehicle &rVehicle)
{
    Vehicle::iterator it = predictions.begin();
    Vehicle temp_vehicle;
    bool vehicle_found = false;
    double max_s = 9999;
    while (it != predictions.end()) {
        double dist = it->s - this->s;
        if (lane == it->get_lane(this->d) && dist > 0) {
            vehicle_found = true;
            
            if (fabs(dist) < max_s) {
                temp_vehicle = *it;
                max_s = dist;
            }
        }
        ++it;
    }
    return vehicle_found;

Vehicle Vehicle::choose_next_state(vector<Vehicle> &predictions)
{
    vector<string> states = successor_states();
    float cost;
    vector<float> costs;
    vector<vector<Vehicle>> final_trajectories;

    for (vector<string>::iterator it = states.begin(); it != states.end(); ++it)
    {
        vector<Vehicle> trajectory = generate_trajectory(*it, predictions);
        if (trajectory.size() != 0)
        {
            cost = calculate_cost(*this, predictions, trajectory);
            costs.push_back(cost);
            final_trajectories.push_back(trajectory);
        }
    }

    vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
    int best_idx = distance(begin(costs), best_cost);

    return final_trajectories[best_idx];
}

vector<string> Vehicle::successor_states()
{
    // Provides the possible next states given the current state for the FSM
    //   discussed in the course, with the exception that lane changes happen
    //   instantaneously, so LCL and LCR can only transition back to KL.
    vector<string> states;
    states.push_back("KL");
    string state = this->state;
    if (state.compare("KL") == 0)
    {
        states.push_back("PLCL");
        states.push_back("PLCR");
    }
    else if (state.compare("PLCL") == 0)
    {
        if (get_lane(this->d) != 2)
        {
            states.push_back("PLCL");
            states.push_back("LCL");
        }
    }
    else if (state.compare("PLCR") == 0)
    {
        if (get_lane(this->d) != 0)
        {
            states.push_back("PLCR");
            states.push_back("LCR");
        }
    }

    // If state is "LCL" or "LCR", then just return "KL"
    return states;
}