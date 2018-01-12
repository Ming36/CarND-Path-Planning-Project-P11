//
//  behavior.hpp
//  Path_Planning
//
//  Created by Student on 1/12/18.
//

#ifndef behavior_hpp
#define behavior_hpp

#include <stdio.h>

#include <string>

struct CarBehavior {
  std::string state;
  int car_ahead_id;
  int target_lane;
  double target_time;
};

#endif /* behavior_hpp */
