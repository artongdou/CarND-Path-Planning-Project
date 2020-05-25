#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>
#include "vehicle.h"

class PathPlanner {

    static double target_speed;
    int target_lane;
    double max_accel;
    double max_decel;

    public:
        std::vector<double> previous_path_x;
        std::vector<double> previous_path_y;
        double end_path_s;
        double end_path_d;
        std::vector<std::vector<double> > sensor_fusion;
        std::vector<Vehicle> cars;
        std::vector<double> map_waypoints_x;
        std::vector<double> map_waypoints_y;
        std::vector<double> map_waypoints_s;

    PathPlanner();
    PathPlanner(std::vector<double> previous_path_x,
                std::vector<double> previous_path_y,
                double end_path_s,
                double end_path_d,
                std::vector<std::vector<double> > sensor_fusion,
                std::vector<double> map_waypoints_x,
                std::vector<double> map_waypoints_y,
                std::vector<double> map_waypoints_s);
    void generate_trajectory(Vehicle &ego, std::vector<double> &next_x_vals, std::vector<double> &next_y_vals);

};

#endif