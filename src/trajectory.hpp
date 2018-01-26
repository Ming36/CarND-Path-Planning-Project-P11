//
//  trajectory.hpp
//  Path_Planning
//
//  Created by Student on 1/12/18.
//

#ifndef trajectory_hpp
#define trajectory_hpp

#include <stdio.h>

#include <vector>
#include "path_common.hpp"
#include "vehicle.hpp"

VehTrajectory GetTrajectory(VehState start_state, double t_tgt,
                            double v_tgt, double d_tgt, double a_tgt,
                            const std::vector<double> &map_interp_s,
                            const std::vector<double> &map_interp_x,
                            const std::vector<double> &map_interp_y);

VehTrajectory GetEgoTrajectory(EgoVehicle &ego_car,
                               const std::map<int, DetectedVehicle> &detected_cars,
                               const std::map<int, std::vector<int>> &car_ids_by_lane,
                               const std::vector<double> &map_interp_s,
                               const std::vector<double> &map_interp_x,
                               const std::vector<double> &map_interp_y);

std::vector<double> CheckTrajFeasibility(VehTrajectory traj);

#endif /* trajectory_hpp */
