#include "costs.h"

using std::vector;

// Cost functions weights configuration
#define SPEED_WEIGHT 2
#define DIST_WEIGHT 3
#define LANE_CHANGE_WEIGHT 1

#define UNSAFE_DIST (30)

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
  if (trajectory[1].v > SPEED_LIMIT) {
    return 1;  // max cost
  } else {
    double diff_d = fabs(trajectory[0].d - trajectory[1].d);
    double diff_v = fabs(SPEED_LIMIT - trajectory[1].v);
    return (1 - exp(-diff_v));
  }
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
  return (1 - exp(-diff_d));
}

/**
 * Function calculate safe distance cost. It panaelizes trajectory that puts the
 * vehicle too close to other vehicles.
 * @param trajectory - vehicle trajectory at {t=0, t=1}
 * @param predictions - vector of all predicted vehicles detected in the
 * surroundings
 * @return speed cost
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
  double min_dist = fmin(fabs(dist_to_veh_ahead), 2 * fabs(dist_to_veh_behind));
  return exp(-fmax(min_dist - UNSAFE_DIST, 0));
}