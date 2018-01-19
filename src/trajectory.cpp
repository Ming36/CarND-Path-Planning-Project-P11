//
//  trajectory.cpp
//  Path_Planning
//
//  Created by Student on 1/12/18.
//

#include "trajectory.hpp"

VehTrajectory GetTrajectory(EgoVehicle &ego_car, double t_tgt,
                            double v_tgt, double d_tgt,
                            const std::vector<double> &map_interp_s,
                            const std::vector<double> &map_interp_x,
                            const std::vector<double> &map_interp_y) {
 
  // Generate S trajectory
  VehState start_state;
  if (ego_car.traj_.states.size() > 0) {
    start_state = ego_car.traj_.states.back();
  }
  else {
    start_state = ego_car.state_;
  }

  double s_est;
  double s_dot_est;
  double s_dot_dot_est;
  
  // Set estimated s, s_dot, s_dot_dot to keep a reasonable JMT with basic kinematics
  const double t_maxa = (v_tgt - start_state.s_dot) / kMaxA;
  
  if (t_maxa > t_tgt) {
    // Cut off target v and a to limit t
    s_dot_est = start_state.s_dot + kMaxA * t_tgt;
    s_dot_dot_est = kMaxA;
  }
  else {
    // Can achieve target speed in time
    s_dot_est = v_tgt;
    s_dot_dot_est = (s_dot_est - start_state.s_dot) / t_tgt;
  }
  
  s_est = start_state.s + start_state.s_dot*t_tgt + 0.5*s_dot_dot_est*t_tgt*t_tgt;
  
  std::vector<double> start_state_s = {start_state.s, start_state.s_dot, start_state.s_dotdot};
  std::vector<double> end_state_s = {s_est, s_dot_est, s_dot_dot_est};

  ego_car.coeffs_JMT_s_ = JMT(start_state_s, end_state_s, t_tgt);
  ego_car.coeffs_JMT_s_dot_ = DiffPoly(ego_car.coeffs_JMT_s_);
  ego_car.coeffs_JMT_s_dotdot_ = DiffPoly(ego_car.coeffs_JMT_s_dot_);
  
  // Generate D trajectory
  
  double d_est = d_tgt;
  double d_dot_est = 0;
  double d_dot_dot_est = 0;
  
  std::vector<double> start_state_d = {start_state.d, start_state.d_dot, start_state.d_dotdot};
  std::vector<double> end_state_d = {d_est, d_dot_est, d_dot_dot_est};
  
  ego_car.coeffs_JMT_d_ = JMT(start_state_d, end_state_d, t_tgt);
  ego_car.coeffs_JMT_d_dot_ = DiffPoly(ego_car.coeffs_JMT_d_);
  ego_car.coeffs_JMT_d_dotdot_ = DiffPoly(ego_car.coeffs_JMT_d_dot_);
  
  // Look up (s,d) vals for each sim cycle time step and convert to raw (x,y)
  VehTrajectory new_traj;
  const int num_pts = t_tgt / kSimCycleTime;

  for (int i = 1; i < num_pts; ++i) { // TODO fix initial index
    double t = i * kSimCycleTime; // idx 0 is 1st point ahead of car
    
    VehState state;
    state.s = EvalPoly(t, ego_car.coeffs_JMT_s_);
    state.s_dot = EvalPoly(t, ego_car.coeffs_JMT_s_dot_);
    state.s_dotdot = EvalPoly(t, ego_car.coeffs_JMT_s_dotdot_);
    state.d = EvalPoly(t, ego_car.coeffs_JMT_d_);
    state.d_dot = EvalPoly(t, ego_car.coeffs_JMT_d_dot_);
    state.d_dotdot = EvalPoly(t, ego_car.coeffs_JMT_d_dotdot_);

    std::vector<double> state_xy = GetHiresXY(state.s, state.d, map_interp_s,
                                            map_interp_x, map_interp_y);
    
    state.x = state_xy[0];
    state.y = state_xy[1];
    
    new_traj.states.push_back(state);
  }
  
  /*
  // Check (x,y)-(s,d) conversion accuracy
  std::vector<double> car_sd = GetHiresFrenet(ego_car.x_, ego_car.y_,
                                              map_hires_s,
                                              map_hires_x,
                                              map_hires_y);
  
  std::vector<double> car_xy = GetHiresXY(car_sd[0], car_sd[1], map_hires_s,
                                          map_hires_x, map_hires_y);
  
  std::cout << "x: " << ego_car.x_ << ", y: " << ego_car.y_
            << ", xy->s: " << ego_car.s_ << ", xy->d: " << ego_car.d_
            << ", sd->x: " << raw_x_vals[0] << ", sd->y: " << raw_y_vals[1];
  
  std::cout << "" << std::endl;
  */
  
  return new_traj;
}

VehTrajectory GetFinalTrajectory(EgoVehicle &ego_car,
                                 const std::vector<double> &map_interp_s,
                                 const std::vector<double> &map_interp_x,
                                 const std::vector<double> &map_interp_y) {
  
  // Set target time and speed
  const double t_tgt = ego_car.tgt_behavior_.tgt_time;
  const double v_tgt = mph2mps(kTargetSpeedMPH); // mph -> m/s
  
  // Set target D based on target lane
  double d_tgt = 1.9 + (ego_car.tgt_behavior_.tgt_lane-1) * 4.0;

  // Calculate initial trajectory
  VehTrajectory traj = GetTrajectory(ego_car, t_tgt, v_tgt, d_tgt,
                                     map_interp_s, map_interp_x, map_interp_y);

  // Check for (x,y) overspeed and recalculate trajectory to compensate
  double v_peak = 0;
  double xy_speed = 0;
  for (int i=0; i < traj.states.size()-1; ++i) {
    xy_speed = (Distance(traj.states[i].x, traj.states[i].y,
                         traj.states[i+1].x, traj.states[i+1].y) / kSimCycleTime);
    if (xy_speed > v_peak) { v_peak = xy_speed; }
  }
  
  //std::cout << "Before: " << mps2mph(v_peak) << std::endl;
  
  if (v_peak > mph2mps(kTargetSpeedMPH)) {
    double speed_adj_ratio = mph2mps(kTargetSpeedMPH) / v_peak;
    //std::cout << "Adj: " << speed_adj_ratio << std::endl;
    
    traj = GetTrajectory(ego_car, t_tgt,
                         (v_tgt * speed_adj_ratio - mph2mps(kSpdAdjOffsetMPH)),
                         d_tgt, map_interp_s, map_interp_x, map_interp_y);
  }
  
  /*
  v_peak = 0;
  for (int i=0; i < traj.x.size()-1; ++i) {
    xy_speed = (Distance(traj.x[i], traj.y[i], traj.x[i+1], traj.y[i+1])
                / kSimCycleTime);
    if (xy_speed > v_peak) { v_peak = xy_speed; }
  }
  std::cout << "After: " << mps2mph(v_peak) << std::endl << std::endl;
  */
  
  return traj;
}
