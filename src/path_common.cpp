//
//  path_common.cpp
//  Path_Planning
//
//  Created by Student on 1/12/18.
//

#include "path_common.hpp"

/**
 * Calculate Cartesian distance between two (x,y) points
 */
double Distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
}

/**
 * Interpolate map waypoints with higher resolution (s at every 1m)
 */
std::vector<std::vector<double>> InterpolateMap(std::vector<double> &map_s,
                                                std::vector<double> &map_x,
                                                std::vector<double> &map_y,
                                                std::vector<double> &map_dx,
                                                std::vector<double> &map_dy,
                                                double s_dist_inc) {
  
  // Add initial map point back to end of map points for wrap-around
  map_s.push_back(kMaxS);
  map_x.push_back(map_x[0]);
  map_y.push_back(map_y[0]);
  map_dx.push_back(map_dx[0]);
  map_dy.push_back(map_dy[0]);
  
  // Make splines by s axis
  tk::spline spline_s_x;
  tk::spline spline_s_y;
  tk::spline spline_s_dx;
  tk::spline spline_s_dy;
  spline_s_x.set_points(map_s, map_x);
  spline_s_y.set_points(map_s, map_y);
  spline_s_dx.set_points(map_s, map_dx);
  spline_s_dy.set_points(map_s, map_dy);
  
  // Interpolate hires map from splines for s at every 1m
  std::vector<double> interp_s;
  std::vector<double> interp_x;
  std::vector<double> interp_y;
  std::vector<double> interp_dx;
  std::vector<double> interp_dy;
  for (double s=0.; s < kMaxS; s = s + s_dist_inc) {
    interp_s.push_back(s);
    interp_x.push_back(spline_s_x(s));
    interp_y.push_back(spline_s_y(s));
    interp_dx.push_back(spline_s_dx(s));
    interp_dy.push_back(spline_s_dy(s));
  }
  
  return {interp_s, interp_x, interp_y, interp_dx, interp_dy};
}

/**
 * Find closest waypoint ahead or behind
 */
int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x,
                    const std::vector<double> &maps_y) {
  
  // TODO Make this a more efficient nearest neighbor search
  
  double closestLen = std::numeric_limits<double>::max(); //large number
  int closestWaypoint = 0;
  
  for (int i = 0; i < maps_x.size(); i++) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = Distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }
  
  return closestWaypoint;
}

/**
 * Transform from Frenet (s,d) coordinates to Cartesian (x,y)
 */
std::vector<double> GetHiresXY(double s, double d,
                               const std::vector<double> &map_hires_s,
                               const std::vector<double> &map_hires_x,
                               const std::vector<double> &map_hires_y) {
  
  /*
  int wp1 = -1;
  while ((s > map_hires_s[wp1+1]) &&
         (wp1 < (int)(map_hires_s.size()-1))) {
    wp1++;
  }
  */
  auto wp1_bst = std::lower_bound(map_hires_s.begin(), map_hires_s.end(), s);

  //std::cout << wp1 << " = " << (wp1_bst-map_hires_s.begin()-1) << std::endl;

  int wp1 = (wp1_bst - map_hires_s.begin() - 1);
  int wp2 = (wp1 + 1) % map_hires_x.size();
  
  double theta_wp = atan2((map_hires_y[wp2] - map_hires_y[wp1]),
                          (map_hires_x[wp2] - map_hires_x[wp1]));
  
  // The x,y,s along the segment between waypoints
  double seg_s = s - map_hires_s[wp1];
  double seg_x = map_hires_x[wp1] + seg_s * cos(theta_wp);
  double seg_y = map_hires_y[wp1] + seg_s * sin(theta_wp);
  
  double theta_perp = theta_wp - pi()/2;
  double x = seg_x + d * cos(theta_perp);
  double y = seg_y + d * sin(theta_perp);
  
  return {x, y};
}

/**
 * Transform from Cartesian (x,y) coordinates to Frenet (s,d)
 */
std::vector<double> GetHiresFrenet(double x, double y,
                                   const std::vector<double> &map_hires_s,
                                   const std::vector<double> &map_hires_x,
                                   const std::vector<double> &map_hires_y) {
  
  // TODO Fix wraparound
  
  int close_wp = ClosestWaypoint(x, y, map_hires_x, map_hires_y);
  int next_wp = (close_wp + 1) % map_hires_x.size();
  //if (next_wp > map_hires_x.size() - 1) { next_wp  = 0; }
  int prev_wp = close_wp - 1;
  if (prev_wp < 0) { prev_wp = map_hires_x.size() - 1; }
  
  double dist_nextwp = Distance(x, y, map_hires_x[next_wp], map_hires_y[next_wp]);
  double dist_prevwp = Distance(x, y, map_hires_x[prev_wp], map_hires_y[prev_wp]);

  int wp1;
  int wp2;
  if (dist_nextwp < dist_prevwp) {
    wp1 = close_wp;
    wp2 = next_wp;
  }
  else {
    wp1 = prev_wp;
    wp2 = close_wp;
  }
  
  // DEBUG
  //std::cout << "prev_wp: " << prev_wp << ", close_wp: " << close_wp << ", next_wp: " << next_wp << std::endl;
  //std::cout << "wp1: " << wp1 << ", wp2: " << wp2 << std::endl;
  
  // Waypoint vector x and y components from prev waypoint to next waypoint
  double vx_wp = map_hires_x[wp2] - map_hires_x[wp1];
  double vy_wp = map_hires_y[wp2] - map_hires_y[wp1];
  
  // Position vector x and y components from prev waypoint to position coord
  double vx_pos = x - map_hires_x[wp1];
  double vy_pos = y - map_hires_y[wp1];
  
  // Find the projection of position vector onto waypoint vector
  double proj_length = ((vx_pos * vx_wp + vy_pos * vy_wp) /
                        (vx_wp * vx_wp + vy_wp * vy_wp));
  double frenet_s = map_hires_s[wp1] + proj_length;
  
  double frenet_d = sqrt(pow(Distance(map_hires_x[wp1], map_hires_y[wp1], x, y), 2)
                         - pow(proj_length, 2));
  
  return {frenet_s, frenet_d};
}

/**
 * Calculate Frenet velocities (s_dot,d_dot) from (vx,vy) and d normal vector
 * based on closest map waypoint's (dx,dy)
 */
std::vector<double> GetFrenetVelocity(double vx, double vy, int closest_wp,
                                      const std::vector<double> &maps_dx,
                                      const std::vector<double> &maps_dy) {
  
  double dx = maps_dx[closest_wp];
  double dy = maps_dy[closest_wp];
  double frenet_d_dot = (vx * dx + vy * dy);
  double frenet_s_dot = sqrt(vx*vx + vy*vy - frenet_d_dot*frenet_d_dot);
  
  //std::cout << "vx:" << vx << ",vy:" << vy << ",dx:" << dx << ",dy:" << dy << ",d_dot:" << frenet_d_dot << ",s_dot:" << frenet_s_dot << std::endl;
  
  return {frenet_s_dot, frenet_d_dot};
}

/**
 * Calculate the Jerk Minimizing Trajectory that connects the initial state
 * to the final state in time t_end.
 */
std::vector<double> JMT(std::vector<double> start, std::vector <double> end,
                        double t_end) {
  /*
   INPUTS
   
   start - the vehicles start location given as a length three array
   corresponding to initial values of [s, s_dot, s_double_dot]
   
   end - the desired end state for vehicle. Like "start" this is a
   length three array.
   
   t_end - The duration, in seconds, over which this maneuver should occur.
   
   OUTPUT
   
   an array of length 6, each value corresponding to a coefficent in the polynomial
   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
   
   EXAMPLE
   
   > JMT( [0, 10, 0], [10, 10, 0], 1)
  
   [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
   */
  
  const double t_end2 = t_end * t_end;
  const double t_end3 = t_end2 * t_end;
  const double t_end4 = t_end3 * t_end;
  const double t_end5 = t_end4 * t_end;
  
  Eigen::MatrixXd A = Eigen::MatrixXd(3, 3);
  A << t_end3, t_end4, t_end5,
       3*t_end2, 4*t_end3, 5*t_end4,
       6*t_end, 12*t_end2, 20*t_end3;
  
  Eigen::MatrixXd B = Eigen::MatrixXd(3,1);
  B << end[0]-(start[0]+start[1]*t_end+0.5*start[2]*t_end2),
       end[1]-(start[1]+start[2]*t_end),
       end[2]-start[2];
  
  Eigen::MatrixXd Ai = A.inverse();
  Eigen::MatrixXd C = Ai * B;
  
  std::vector <double> result = {start[0], start[1], 0.5*start[2]};
  for(int i = 0; i < C.size(); i++) {
    result.push_back(C.data()[i]);
  }
  
  return result;
}

/**
 * Evaluate a polynomial with form f(x) = a0 + a1*x + a2*x^2 + ... at x
 */
double EvalPoly(double x, std::vector<double> coeffs) {
  double y = 0;
  for (int i=0; i < coeffs.size(); ++i) {
    y += coeffs[i] * pow(x, i);
  }
  return y;
}

/**
 * Differentiate a polynomial with form f(x) = a0 + a1*x + a2*x^2 + ... to get
 * the derivative polynomial's coefficients
 */
std::vector<double> DiffPoly(std::vector<double> coeffs) {
  std::vector<double> diff_coeffs;
  for (int i=1; i < coeffs.size(); ++i) {
    diff_coeffs.push_back(i*coeffs[i]);
  }
  return diff_coeffs;
}

