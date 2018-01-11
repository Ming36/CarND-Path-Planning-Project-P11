//
//  prediction.cpp
//  Path_Planning
//
//  Created by Student on 1/9/18.
//

#include "prediction.hpp"
#include "math.h"
/**
 * Initializes Vehicle
 */

Vehicle::Vehicle(int veh_id) {
  this->veh_id = veh_id;
}

Vehicle::~Vehicle() { }

void Vehicle::UpdateState(double x, double y, double s, double d, double s_dot, double d_dot, double ego_s, double ego_d) {

  // Store previous values to calculate s_dot_dot and d_dot_dot later
  this->s_prev = this->s;
  this->s_dot_prev = this->s_dot;
  this->d_prev = this->d;
  this->d_dot_prev = this->d_dot;
  
  // Update state values
  this->x = x;
  this->y = y;
  this->s = s;
  this->d = d;
  this->s_dot = s_dot;
  this->d_dot = d_dot;
  
  // Calculate s_dot_dot and d_dot_dot from stored previous data
  // Equation: s_dot_dot = s_dot_prev*(s_dot - s_dot_prev) + 0.5*(s_dot - s_dot_prev)^2 / (s - s_prev)
  if ((s - this->s_prev) > 0.0001) {
    this->s_dot_dot = (this->s_dot_prev * (s_dot - this->s_dot_prev)
                       + 0.5 * pow(s_dot - this->s_dot_prev, 2) / (s - this->s_prev));
  }
  else {
    this->s_dot_dot = 0;
  }
  
  if ((d - this->d_prev) > 0.0001) {
    this->d_dot_dot = (this->d_dot_prev * (d_dot - this->d_dot_prev)
                       + 0.5 * pow(d_dot - this->d_dot_prev, 2) / (d - this->d_prev));
  }
  else {
    this->d_dot_dot = 0;
  }

  // Calculate relative (s,d) from ego car
  this->s_rel = s - ego_s;
  this->d_rel = d - ego_d;
  
  // Detect lane
  int lane = 0;
  double n = 0;
  constexpr double kLaneWidth = 4.0; // m
  while (d > n * kLaneWidth) {
    lane++;
    n = n + 1.0;
  }
  this->lane = lane;
  
}

void Vehicle::PredictBehavior(){
  this->intent = "KL";
}
void Vehicle::PredictTrajectory(){
  this->trajectory_s = {};
  this->trajectory_d = {};
}

/*
std::vector<Vehicle> Vehicle::choose_next_state(std::map<int, std::vector<Vehicle>> predictions) {
  *
   Here you can implement the transition_function code from the Behavior Planning Pseudocode
   classroom concept. Your goal will be to return the best (lowest cost) trajectory corresponding
   to the next state.
   
   INPUT: A predictions map. This is a map of vehicle id keys with predicted
   vehicle trajectories as values. Trajectories are a vector of Vehicle objects representing
   the vehicle at the current timestep and one timestep in the future.
   OUTPUT: The the best (lowest cost) trajectory corresponding to the next ego vehicle state.
   
   *
  std::vector<std::string> states = successor_states();
  float cost;
  std::vector<float> costs;
  std::vector<std::string> final_states;
  std::vector<std::vector<Vehicle>> final_trajectories;
  
  for (std::vector<std::string>::iterator it = states.begin(); it != states.end(); ++it) {
    std::vector<Vehicle> trajectory = generate_trajectory(*it, predictions);
    if (trajectory.size() != 0) {
      cost = calculate_cost(*this, predictions, trajectory);
      costs.push_back(cost);
      final_trajectories.push_back(trajectory);
    }
  }
  
  std::vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
  int best_idx = distance(begin(costs), best_cost);
  return final_trajectories[best_idx];
}


float calculate_cost(const Vehicle & vehicle, const std::map<int, std::vector<Vehicle>> & predictions, const std::vector<Vehicle> & trajectory) {
  *
   Sum weighted cost functions to get total cost for trajectory.
   *
  std::map<std::string, float> trajectory_data = get_helper_data(vehicle, trajectory, predictions);
  float cost = 0.0;
  
  //Add additional cost functions here.
  std::vector< std::function<float(const Vehicle & , const std::vector<Vehicle> &, const std::map<int, std::vector<Vehicle>> &, std::map<std::string, float> &)>> cf_list = {goal_distance_cost, inefficiency_cost};
  std::vector<float> weight_list = {REACH_GOAL, EFFICIENCY};
  
  for (int i = 0; i < cf_list.size(); i++) {
    float new_cost = weight_list[i]*cf_list[i](vehicle, trajectory, predictions, trajectory_data);
    cost += new_cost;
  }
  
  return cost;
}
*/
