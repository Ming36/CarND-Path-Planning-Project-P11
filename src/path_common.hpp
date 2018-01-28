//
//  path_common.hpp
//  Path_Planning
//
//  Created by Student on 1/12/18.
//

#ifndef path_common_hpp
#define path_common_hpp

#include <stdio.h>

#include <iostream>
#include <math.h>
#include <vector>
#include "Eigen-3.3/Eigen/Dense"
#include "spline.h"

/**
 * Constant parameters
 */

// Simulation
constexpr double kSimCycleTime = 0.02; // sec
constexpr double kSensorRange = 100.; // m

// Track
constexpr double kMaxS = 6945.554; // max before track wraps back to 0
constexpr double kMapInterpInc = 1.0; // m increment in s
constexpr double kLaneWidth = 3.9; // m
constexpr int kNumLanes = 3; // # of lanes in the road

// Prediction
constexpr double kLatVelLaneChange = (5.0) / 2.23694; // mph -> m/s

// Trajectory
constexpr int kPathCycleTimeMS = 200; // ms, cycle time to update path
constexpr double kPredictTime = 3.0; // sec
constexpr double kNewPathTime = 2.0; // sec
constexpr double kPathBufferTime = 0.4; // sec, append traj after path is this short
constexpr double kMinTrajPntDist = (3.) / 2.23694 * kSimCycleTime; // mph -> m, smooth standing start
constexpr double kMaxA = 8.0; // max a for constant accel approximation to keep peak <10m/s2
constexpr double kSpdAdjOffset = (1.) / 2.23694; // mph -> m/s
constexpr double kAccAdjOffset = 0.5; // m/s^2

constexpr double kCollisionSThresh = 6.0;
constexpr double kCollisionDThresh = 3.0; // 2.5 may be better
constexpr int kEvalRiskStep = 5; // time step interval to check risk

// Behavior
constexpr double kTargetSpeed = (49.) / 2.23694; // mph -> m/s

constexpr double kTgtStartFollowDist = 40.; // m
constexpr double kTgtFollowDist = 15.; // m
constexpr double kTgtMinFollowDist = 10.; // m
constexpr double kTgtMinFollowGain = 1.; // speed slope gain multiplier
constexpr double kTgtMinSpeed = (5.) / 2.23694; // mph -> m/s, min target speed
constexpr double kTgtMinFollowTime = kNewPathTime; // shortened trajectory time
constexpr double kTgtSpeedDec = (5.) / 2.23694; // mph -> m/s, for Plan LC

constexpr double kCostDistAhead = 5.0;
constexpr double kCostSpeedAhead = 4.0;
constexpr double kCostSpeedBehind = 0.0;
constexpr double kCostChangeLanes = 1.0;
constexpr double kCostFreqLaneChange = 1.0;
constexpr double kCostSideGap = 1.0; // 10

constexpr int kCounterFreqLaneChange = 15; // path cycles
constexpr double kLaneChangeMinGap = 8.; // m

/**
 * Basic parameter helpers
 */
constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }
inline double mps2mph(double x) { return x * 2.23694; }
inline double mph2mps(double x) { return x / 2.23694; }
inline double mps2pointdist(double x) { return x * 0.020; }
inline double sq(double x) {return x * x;}

inline double tgt_lane2tgt_d(int tgt_lane) {
  return (kLaneWidth/2) + (tgt_lane-1)*kLaneWidth;
}

double Distance(double x1, double y1, double x2, double y2);

std::vector<std::vector<double>> InterpolateMap(std::vector<double> &map_s,
                                                std::vector<double> &map_x,
                                                std::vector<double> &map_y,
                                                std::vector<double> &map_dx,
                                                std::vector<double> &map_dy,
                                                double s_dist_inc);

int ClosestWaypoint(double x, double y, const std::vector<double> &map_x,
                    const std::vector<double> &map_y);

std::vector<double> GetHiResXY(double s, double d,
                               const std::vector<double> &map_s,
                               const std::vector<double> &map_x,
                               const std::vector<double> &map_y);

std::vector<double> GetHiResFrenet(double x, double y,
                                   const std::vector<double> &map_s,
                                   const std::vector<double> &map_x,
                                   const std::vector<double> &map_y);

std::vector<double> GetFrenetVelocity(double vx, double vy, int closest_wp,
                                      const std::vector<double> &map_dx,
                                      const std::vector<double> &map_dy);

std::vector<double> JMT(std::vector< double> start, std::vector <double> end,
                        double t_end);

double EvalPoly(double x, std::vector<double> coeffs);

std::vector<double> DiffPoly(std::vector<double> coeffs);

double LogCost(double x, double x_saturate);

#endif /* path_helper_hpp */
