#ifndef COSTS_H
#define COSTS_H

#include <vector>

#include "helpers.h"
#include "math.h"
#include "vehicle.h"

double calculate_cost(std::vector<Vehicle> &trajectory,
                      std::vector<Vehicle> &predictions);
double speed_cost(std::vector<Vehicle> &trajectory,
                  std::vector<Vehicle> &predictions);
double lane_change_cost(std::vector<Vehicle> &trajectory,
                        std::vector<Vehicle> &predictions);
double safe_distance_cost(std::vector<Vehicle> &trajectory,
                          std::vector<Vehicle> &predictions);

#endif