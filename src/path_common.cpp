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
  return sqrt(sq(x2-x1) + sq(y2-y1));
}

/**
 * Interpolate map waypoints with higher resolution by Frenet s increments
 */
std::vector<std::vector<double>> InterpolateMap(std::vector<double> map_s,
                                                std::vector<double> map_x,
                                                std::vector<double> map_y,
                                                std::vector<double> map_dx,
                                                std::vector<double> map_dy,
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
  
  // Interpolate map from splines for s at every s_dist_inc increment
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
 *
 * Note: This is not efficient, might be improved by k-d tree search
 */
int ClosestWaypoint(double x, double y, const std::vector<double> &map_x,
                    const std::vector<double> &map_y) {
  
  double closestLen = std::numeric_limits<double>::max(); //large number
  int closestWaypoint = 0;
  
  for (int i = 0; i < map_x.size(); ++i) {
    double wp_x = map_x[i];
    double wp_y = map_y[i];
    double dist = Distance(x,y,wp_x,wp_y);
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
std::vector<double> GetHiResXY(double s, double d,
                               const std::vector<double> &map_s,
                               const std::vector<double> &map_x,
                               const std::vector<double> &map_y) {
  
  // Wrap around s
  s = std::fmod(s, kMaxS);

  // Find 2 waypoints before s and 2 waypoints after s for angular interpolation
  auto it_wp1_search = std::lower_bound(map_s.begin(), map_s.end(), s);
  int wp1 = (it_wp1_search - map_s.begin() - 1); // wp before s
  int wp2 = (wp1 + 1) % map_s.size(); // wp after s
  int wp3 = (wp2 + 1) % map_s.size(); // wp 2nd after s
  int wp0 = wp1 - 1; // wp 2nd before s
  if (wp0 < 0) { wp0 = map_s.size() - 1; } // wrap around backwards

  // Use angle between wp1-wp2 to derive segment vector at distance s from wp1
  double theta_wp = atan2((map_y[wp2] - map_y[wp1]),
                          (map_x[wp2] - map_x[wp1]));
  
  // The (x,y,s) along the segment vector between wp1 and wp2
  double seg_s = s - map_s[wp1];
  double seg_x = map_x[wp1] + seg_s * cos(theta_wp);
  double seg_y = map_y[wp1] + seg_s * sin(theta_wp);

  // Interpolate theta at s based on the distance between wp1 (with ave angle
  // from wp0 before and wp2 after) and wp2 (with ave angle from wp1 before
  // and wp3 after)
  double theta_wp1ave = atan2((map_y[wp2] - map_y[wp0]),
                              (map_x[wp2] - map_x[wp0]));
  
  double theta_wp2ave = atan2((map_y[wp3] - map_y[wp1]),
                              (map_x[wp3] - map_x[wp1]));
  
  double s_interp = (s - map_s[wp1]) / (map_s[wp2] - map_s[wp1]);
  
  double cos_interp = ((1-s_interp) * cos(theta_wp1ave)
                         + s_interp * cos(theta_wp2ave));
  
  double sin_interp = ((1-s_interp) * sin(theta_wp1ave)
                         + s_interp * sin(theta_wp2ave));
  
  double theta_interp = atan2(sin_interp, cos_interp);
  
  // Use interpolated theta to calculate final (x,y) at d offset from the
  // segment vector
  double theta_perp = theta_interp - pi()/2;
  double x = seg_x + d * cos(theta_perp);
  double y = seg_y + d * sin(theta_perp);
  
  return {x, y};
}

/**
 * Transform from Cartesian (x,y) coordinates to Frenet (s,d)
 */
std::vector<double> GetHiResFrenet(double x, double y,
                                   const std::vector<double> &map_s,
                                   const std::vector<double> &map_x,
                                   const std::vector<double> &map_y) {
  
  // Get closest pair of waypoints to (x,y)
  int close_wp = ClosestWaypoint(x, y, map_x, map_y);
  int next_wp = (close_wp + 1) % map_x.size(); // wrap around end
  int prev_wp = close_wp - 1;
  if (prev_wp < 0) { prev_wp = map_x.size() - 1; } // wrap around beginning
  double dist_nextwp = Distance(x, y, map_x[next_wp], map_y[next_wp]);
  double dist_prevwp = Distance(x, y, map_x[prev_wp], map_y[prev_wp]);
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
  
  // Waypoint vector x and y components from prev waypoint to next waypoint
  double vx_wp = map_x[wp2] - map_x[wp1];
  double vy_wp = map_y[wp2] - map_y[wp1];
  
  // Position vector x and y components from prev waypoint to position coord
  double vx_pos = x - map_x[wp1];
  double vy_pos = y - map_y[wp1];
  
  // Find the scalar projection of position vector onto waypoint vector
  double norm_wp = sqrt(sq(vx_wp) + sq(vy_wp));
  double scalar_proj = ((vx_pos * vx_wp + vy_pos * vy_wp) / norm_wp);
  
  // Calculate d using distance from projection to the position coord, with
  // special cases where the projection is outside of waypoint vector ends
  double frenet_d;
  if (scalar_proj < 0.) {
    // Projection to position coord goes behind wp1
    scalar_proj = 0.; // limit scalar proj to endpoint for calculating s
    frenet_d = Distance(map_x[wp1], map_y[wp1], x, y);
  }
  else if (scalar_proj > norm_wp) {
    // Projection to position coord goes past wp2
    scalar_proj = norm_wp; // limit scalar proj to endpoint for calculating s
    frenet_d = Distance(map_x[wp2], map_y[wp2], x, y);
  }
  else {
    // Projection to position coord is within wp1-wp2
    double dist_wp1_pos = Distance(map_x[wp1], map_y[wp1], x, y);
    frenet_d = sqrt(sq(dist_wp1_pos) - sq(scalar_proj)); // Pythagorean
  }
  
  // Calculate s using scalar_proj limited between waypoint pair (0, norm_wp)
  double frenet_s = map_s[wp1] + scalar_proj;
  
  return {frenet_s, frenet_d};
}

/**
 * Calculate Frenet velocities (s_dot,d_dot) from (vx,vy) and d normal vector
 * based on closest map waypoint's (dx,dy)
 */
std::vector<double> GetFrenetVelocity(double vx, double vy, int closest_wp,
                                      const std::vector<double> &map_dx,
                                      const std::vector<double> &map_dy) {
  
  double dx = map_dx[closest_wp];
  double dy = map_dy[closest_wp];
  double frenet_d_dot = (vx * dx + vy * dy);
  double frenet_s_dot = sqrt(sq(vx) + sq(vy) - sq(frenet_d_dot));
  
  return {frenet_s_dot, frenet_d_dot};
}

/**
 * Calculate the Jerk Minimizing Trajectory that connects the start state
 * to the end state in time t_end.  The state variables are [s, s_dot, s_dotdot]
 * and the output is a vector of 5th order polynomial coefficients a0-a5:
 * s(t) = a0 + a1 * t + a2 * t^2 + a3 * t^3 + a4 * t^4 + a5 * t^5
 */
std::vector<double> JMT(std::vector<double> start, std::vector<double> end,
                        double t_end) {
  
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
  
  std::vector<double> result = {start[0], start[1], 0.5*start[2]};
  for(int i = 0; i < C.size(); ++i) {
    result.push_back(C.data()[i]);
  }
  
  return result; // [a0, a1, a2, a3, a4, a5]
}

/**
 * Evaluate a polynomial with form "y = a0 + a1*x + a2*x^2 + ..." at x
 */
double EvalPoly(double x, std::vector<double> coeffs) {
  double y = 0;
  for (int i = 0; i < coeffs.size(); ++i) {
    y += coeffs[i] * pow(x, i);
  }
  return y;
}

/**
 * Differentiate a polynomial with form y = a0 + a1*x + a2*x^2 + ... to get
 * the derivative polynomial's coefficients
 */
std::vector<double> DiffPoly(std::vector<double> coeffs) {
  std::vector<double> diff_coeffs;
  for (int i = 1; i < coeffs.size(); ++i) {
    diff_coeffs.push_back(i*coeffs[i]);
  }
  return diff_coeffs;
}

/**
 * A logistic function used for cost functions that returns a value
 * between 0 and 1 for x in the range [0, infinity] with saturating near 1
 * at x = x_saturate.
 */
double LogCost(double x, double x_saturate) {
  if (abs(x_saturate) >= 1.0) {
    return (2. / (1. + exp(-5. / x_saturate * x )) - 1.);
  }
  else {
    return (2. / (1. + exp(-5. * x)) - 1.);
  }
}
