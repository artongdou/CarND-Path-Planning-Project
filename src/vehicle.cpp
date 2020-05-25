#include "vehicle.h"

Vehicle::Vehicle() {}

Vehicle::Vehicle(int id) {
    this->id = id;
}

Vehicle::Vehicle(int id, double x, double y, double yaw, double s, double d, double vx, double vy) {
    this->x = x;
    this->y = y;
    this->yaw = yaw;
    this->s = s;
    this->d = d;
    this->vx = vx;
    this->vy = vy;
}