#ifndef VEHICLE_H
#define VEHICLE_H

#include <string>
#include <vector>

class Vehicle {
 public:
  int id;
  double x, y, yaw;
  double s, d;
  double v;
  std::string state;

  Vehicle();
  Vehicle(int id);
  Vehicle(int id, double x, double y, double yaw, double s, double d, double v);
  int get_lane();
  int get_lane(double d);
  Vehicle generate_predictions(double dt);
  bool get_vehicle_ahead(int lane, std::vector<Vehicle> &predictions,
                         Vehicle &rVehicle);
  bool get_vehicle_behind(int lane, std::vector<Vehicle> &predictions,
                          Vehicle &rVehicle);
  std::vector<Vehicle> choose_next_state(std::vector<Vehicle> &predictions);
  std::vector<std::string> successor_states();
  std::vector<Vehicle> keep_lane_trajectory(std::vector<Vehicle> &preditions);
  std::vector<Vehicle> lane_change_trajectory(
      std::string state, std::vector<Vehicle> &predictions);
};

double calculate_cost(std::vector<Vehicle> &trajectory,
                      std::vector<Vehicle> &predictions);
double speed_cost(std::vector<Vehicle> &trajectory,
                  std::vector<Vehicle> &predictions);
double safe_distance_cost(std::vector<Vehicle> &trajectory,
                          std::vector<Vehicle> &predictions);
double lane_change_cost(std::vector<Vehicle> &trajectory,
                        std::vector<Vehicle> &predictions);

#define SPEED_LIMIT (mph2mps(49))
#define MAX_JERK (7)

#endif