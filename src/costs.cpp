#include "costs.h"

#include <iostream>

using std::cout;
using std::endl;
using std::vector;

// Cost functions weights configuration
#define SPEED_WEIGHT 15
#define DIST_WEIGHT 40
#define LANE_CHANGE_WEIGHT 4

#define UNSAFE_DIST (10)

/**
 * Function calculate costs based on given trajectory at {t=0, t=1}
 * @param trajectory - vehicle trajectory at {t=0, t=1}
 * @param predictions - vector of all predicted vehicles detected in the
 * surroundings
 * @return total cost
 */
double calculate_cost(vector<Vehicle> &trajectory,
                      vector<Vehicle> &predictions) {
  vector<std::function<double(vector<Vehicle> &, vector<Vehicle> &)>> cf_list =
      {speed_cost, safe_distance_cost, lane_change_cost};
  vector<double> weight_list = {SPEED_WEIGHT, DIST_WEIGHT, LANE_CHANGE_WEIGHT};
  double cost = 0;
  for (int i = 0; i < cf_list.size(); ++i) {
    double new_cost = weight_list[i] * cf_list[i](trajectory, predictions);
    cost += new_cost;
  }

  return cost;
}

/**
 * Function calculate speed cost. Closer to the speed limit yields lower cost,
 * but exceeding speed limit will be max cost.
 * @param trajectory - vehicle trajectory at {t=0, t=1}
 * @param predictions - vector of all predicted vehicles detected in the
 * surroundings
 * @return speed cost
 */
double speed_cost(vector<Vehicle> &trajectory, vector<Vehicle> &predictions) {
  double cost;
  if (trajectory[1].v > SPEED_LIMIT) {
    cost = 1;  // max cost
  } else {
    double diff_d = fabs(trajectory[0].d - trajectory[1].d);
    double diff_v = fabs(SPEED_LIMIT - trajectory[1].v) / 4;
    cost = (1 - exp(-diff_v));
  }
  cout << "      "
       << "speed cost= " << cost << endl;
  return cost;
}

/**
 * Function to calculate lane change cost. It panelizes the distance travel at d
 * direction.
 * @param trajectory - vehicle trajectory at {t=0, t=1}
 * @param predictions - vector of all predicted vehicles detected in the
 * surroundings
 * @return lane change cost
 */
double lane_change_cost(vector<Vehicle> &trajectory,
                        vector<Vehicle> &predictions) {
  double diff_d = fabs(trajectory[0].d - trajectory[1].d);
  double cost = 1 - exp(-diff_d);
  cout << "      "
       << "lane change cost= " << cost << endl;
  return cost;
}

/**
 * Function calculate safe distance cost. It panalizes trajectory that puts the
 * vehicle too close to other vehicles.
 * @param trajectory - vehicle trajectory at {t=0, t=1}
 * @param predictions - vector of all predicted vehicles detected in the
 * surroundings
 * @return safe distance cost
 */
double safe_distance_cost(vector<Vehicle> &trajectory,
                          vector<Vehicle> &predictions) {
  double dist_to_veh_ahead = 9999999;
  double dist_to_veh_behind = 9999999;
  Vehicle veh_ahead, veh_behind;
  if (trajectory[1].get_vehicle_ahead(trajectory[1].get_lane(), predictions,
                                      veh_ahead)) {
    dist_to_veh_ahead = veh_ahead.s - trajectory[1].s;
  }
  if (trajectory[1].get_vehicle_behind(trajectory[1].get_lane(), predictions,
                                       veh_behind)) {
    dist_to_veh_behind = -veh_behind.s + trajectory[1].s;
  }
  double cost;
  double min_dist = fmin(fabs(dist_to_veh_ahead), fabs(dist_to_veh_behind));
  if (min_dist <= UNSAFE_DIST) {
    cost = 1;
  } else {
    cost = exp(-fabs(min_dist - UNSAFE_DIST) / 10);
  }
  cout << "dist to veh ahead: " << dist_to_veh_ahead << endl;
  cout << "      "
       << "distance cost= " << cost << endl;
  return cost;
}