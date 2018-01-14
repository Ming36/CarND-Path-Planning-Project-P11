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
#include "path_helper.hpp"
#include "vehicle.hpp"

VehTrajectory GetTrajectory(double car_s, double car_d,
                            const std::vector<double> &maps_x,
                            const std::vector<double> &maps_y,
                            const std::vector<double> &maps_s);

#endif /* trajectory_hpp */
