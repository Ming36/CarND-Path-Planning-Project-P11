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

void GetTrajectory(EgoVehicle &ego_car,
                   const std::vector<double> &map_hires_s,
                   const std::vector<double> &map_hires_x,
                   const std::vector<double> &map_hires_y);

#endif /* trajectory_hpp */
