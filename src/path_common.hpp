//
//  path_common.hpp
//  Path_Planning
//
//  Created by Student on 1/12/18.
//

#ifndef path_common_hpp
#define path_common_hpp

#include <stdio.h>
#include <math.h>
#include <iostream>
#include <string>
#include <vector>
#include <deque>
#include <map>
#include "Eigen-3.3/Eigen/Dense"
#include "spline.h"

/**
 * Constant parameters
 */

// Debug logging
constexpr int kDBGMain = 0; // 1=Basic, 2=Telemetry, 3=Every Sim Loop
constexpr int kDBGVehicle = 0;
constexpr int kDBGSensorFusion = 0;
constexpr int kDBGPrediction = 0;
constexpr int kDBGBehavior = 0;
constexpr int kDBGTrajectory = 0;

// Simulation and Track
constexpr double kSimCycleTime = 0.02; // sec
constexpr double kMaxS = 6945.554; // m, max S before track wraps back to 0
constexpr double kMapInterpInc = 1.; // m, interpolated increment in Frenet S
constexpr double kLaneWidth = 3.9; // m, width per lane
constexpr int kNumLanes = 3; // # of lanes in the road

// Main Path Planner
constexpr int kPathCycleTimeMS = 200; // ms, path planner cycle time
constexpr double kSensorRange = 100.; // m, limit detected cars within range

// Prediction
constexpr double kLatVelLaneChange = (5.) / 2.23694; // (mph)->m/s to judge LC
constexpr double kPredictTime = 1.5; // sec, time to predict car paths

// Behavior
constexpr double kCostDistAhead = 5.; // cost gain
constexpr double kCostSpeedAhead = 7.; // cost gain
constexpr double kCostSpeedBehind = 7.; // cost gain
constexpr double kCostChangeLanes = 0.8; // cost gain
constexpr double kCostFreqLaneChange = 1.; // cost gain
constexpr int kCounterFreqLaneChange = 15; // # of path cycles for counter reset
constexpr double kRelSpeedBehind = (10.) / 2.23694; // (mph)->m/s
constexpr double kLaneChangeMinGap = 10.; // m, min side gap for lane change
constexpr double kTargetSpeed = (49.) / 2.23694; // (mph)->m/s, base target
constexpr double kTgtMinSpeed = (0.) / 2.23694; // (mph)->m/s, min target speed
constexpr double kTgtStartFollowDist = 40.; // m, dist to start reducing speed
constexpr double kTgtFollowDist = 14.; // m, dist to match car ahead's speed
constexpr double kTgtMinFollowDist = 11.; // m, dist to slow down faster
constexpr double kTgtMinFollowSpeedDec = (10.) / 2.23694; // (mph)->m/s to slow
constexpr double kPLCTgtSpeedDec = (20.) / 2.23694; // (mph)->m/s, to find gap
constexpr double kPLCCloseDist = 11.; // m, dist to judge close car during PLC

// Trajectory
constexpr double kPathBufferTime = 0.5; // sec, duration of prev path buffer
constexpr double kNewPathTime = 2.5; // sec, duration of new planned path
constexpr double kMinTrajPntDist = (3.) / 2.23694 * kSimCycleTime; // (mph)->m
constexpr double kMaxA = 8.; // m/s^2, target max accel to keep peak < 10m/s^2
constexpr double kSpdAdjOffset = (2.) / 2.23694; // (mph)->m/s, spd adj offset
constexpr double kAccAdjOffset = 1.; // m/s^2, accel adj offset
constexpr int kAccelAveSamples = 10; // # samples for smoothing ave accel
constexpr double kCollisionSThresh = 8.; // m, gap S to judge collision risk
constexpr double kCollisionDThresh = 3.; // m, gap D to judge collision risk
constexpr int kEvalRiskStep = 10; // # time steps for risk check interval
constexpr int kTrajGenNum = 5; // # of possible traj variations to sample from
constexpr double kRandSpdMean = (5.) / 2.23694; // (mph)->m/s, speed adj mean
constexpr double kRandSpdDev = (2.) / 2.23694; // (mph)->m/s, speed adj std dev
constexpr double kRandTimeMean = 0.; // sec, path time adj mean
constexpr double kRandTimeDev = 0.6; // sec, path time adj std dev
constexpr double kMinTrajTime = 1.5; // sec, guard min traj time
constexpr double kTrajCostRisk = 10.; // cost gain for traj collision risk
constexpr double kTrajCostDeviation = 1.; // cost gain for deviation from base
constexpr double kTrajCostThresh = 20.; // cost thresh to judge traj risk
constexpr double kBackupTgtSpeedDec = (10.) / 2.23694; // (mph)->m/s spd steps

/**
 * Basic parameter helpers
 */
constexpr double pi() { return M_PI; }
inline double mps2mph(double x) { return x * 2.23694; }
inline double sq(double x) { return x * x; }

inline double tgt_lane2tgt_d(int tgt_lane) {
  return (kLaneWidth / 2) + (tgt_lane - 1) * kLaneWidth;
}

double Distance(double x1, double y1, double x2, double y2);

std::vector<std::vector<double>> InterpolateMap(std::vector<double> map_s,
                                                std::vector<double> map_x,
                                                std::vector<double> map_y,
                                                std::vector<double> map_dx,
                                                std::vector<double> map_dy,
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

std::vector<double> JMT(std::vector< double> start, std::vector<double> end,
                        double t_end);

double EvalPoly(double x, std::vector<double> coeffs);

std::vector<double> DiffPoly(std::vector<double> coeffs);

double LogCost(double x, double x_saturate);

#endif /* path_helper_hpp */
