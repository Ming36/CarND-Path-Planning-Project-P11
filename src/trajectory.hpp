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

VehTrajectory GetFinalTrajectory(EgoVehicle &ego_car,
                                 const std::vector<double> &map_interp_s,
                                 const std::vector<double> &map_interp_x,
                                 const std::vector<double> &map_interp_y);

VehTrajectory GetTrajectory(EgoVehicle &ego_car, double target_speed,
                             const std::vector<double> &map_interp_s,
                             const std::vector<double> &map_interp_x,
                             const std::vector<double> &map_interp_y);

#endif /* trajectory_hpp */
