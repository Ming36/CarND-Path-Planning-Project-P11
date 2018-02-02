//
//  trajectory.hpp
//  Path_Planning
//
//  Created by Student on 1/12/18.
//

#ifndef trajectory_hpp
#define trajectory_hpp

#include <stdio.h>
#include <random>
#include "vehicle.hpp"

VehTrajectory GetBufferTrajectory(int idx_current_pt,
                                  VehTrajectory prev_ego_traj);

VehTrajectory GetEgoTrajectory(const EgoVehicle &ego_car,
                         const std::map<int, DetectedVehicle> &detected_cars,
                         const std::map<int, std::vector<int>> &car_ids_by_lane,
                         const std::vector<double> &map_interp_s,
                         const std::vector<double> &map_interp_x,
                         const std::vector<double> &map_interp_y);

VehTrajectory GetTrajectory(VehState start_state, double t_tgt,
                            double v_tgt, double d_tgt, double a_tgt,
                            const std::vector<double> &map_interp_s,
                            const std::vector<double> &map_interp_x,
                            const std::vector<double> &map_interp_y);

std::vector<double> CheckTrajFeasibility(const VehTrajectory traj);

double EvalTrajCost(const VehTrajectory traj, const EgoVehicle &ego_car,
                    const std::map<int, DetectedVehicle> &detected_cars);

#endif /* trajectory_hpp */
