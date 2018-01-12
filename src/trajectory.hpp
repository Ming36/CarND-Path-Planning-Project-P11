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

struct Trajectory {
  std::vector<double> coord1;
  std::vector<double> coord2;
};

Trajectory GetTrajectory(double car_s, double car_d,
                                               const std::vector<double> &maps_x,
                                               const std::vector<double> &maps_y,
                                               const std::vector<double> &maps_s);

#endif /* trajectory_hpp */
