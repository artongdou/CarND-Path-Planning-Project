#ifndef VEHICLE_H
#define VEHICLE_H

class Vehicle
{
public:
    int id;
    double x, y, yaw;
    double s, d;
    double vx, vy;

    Vehicle();
    Vehicle(int id);
    Vehicle(int id, double x, double y, double yaw, double s, double d, double vx, double vyß);
};

#endif