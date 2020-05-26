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
    Vehicle generate_predictions(double dt);
    int get_lane();
    std::vector<std::string> successor_states();
};

#endif