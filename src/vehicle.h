#ifndef VEHICLE_H
#define VEHICLE_H

#include <string>
#include <vector>

class Vehicle
{
public:
    int id;
    double x, y, yaw;
    double s, d;
    double vx, vy;
    std::string state;

    Vehicle();
    Vehicle(int id);
    Vehicle(int id, double x, double y, double yaw, double s, double d, double vx, double vy);
    int get_lane(double d);
    Vehicle generate_predictions(double dt);
    bool get_vehicle_ahead(int lane, std::vector<Vehicle> &predictions, Vehicle &rVehicle);
    bool get_vehicle_behind(int lane, std::vector<Vehicle> &predictions, Vehicle &rVehicle);
    Vehicle choose_next_state(std::vector<Vehicle> &predictions);
    std::vector<std::string> successor_states();
};

#endif