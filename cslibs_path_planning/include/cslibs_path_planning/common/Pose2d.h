/*
 * Pose2d.h
 *
 *  Created on: Apr 4, 2011
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef POSE2D_H
#define POSE2D_H

#include <cmath>
#include <iostream>

namespace lib_path {

class Pose2d
{
public:
  Pose2d();
  Pose2d(double x, double y, double theta);

  /**
     * Computes the distance to the given point
     *
     * @param rhs point to compute the distance to
     */
  double distance_to(const Pose2d &rhs) const;

  /**
     * Computes the distance to the origin
     */
  double distance_to_origin() const;

  bool isEqual( const Pose2d &p, const double dist_eps, const double theta_eps ) const;

public:
  // x-coordinate
  double x;
  // y-coordinate
  double y;
  // orientation
  double theta;


  /*
     * Operators
     */
public:
  Pose2d operator - (const Pose2d &rhs) const{
    Pose2d p;
    p.x = x - rhs.x;
    p.y = y - rhs.y;
    return p;
  }

  Pose2d operator + (const Pose2d &rhs) const{
    Pose2d p;
    p.x = x + rhs.x;
    p.y = y + rhs.y;
    return p;
  }

  Pose2d operator / (float rhs) const{
    Pose2d p;
    p.x = x / rhs;
    p.y = y / rhs;
    return p;
  }
};

inline Pose2d operator * (const Pose2d &pose, float factor) {
  Pose2d p;
  p.x = pose.x * factor;
  p.y = pose.y * factor;
  return p;
}

inline Pose2d operator * (float factor, const Pose2d &pose) {
  Pose2d p;
  p.x = pose.x * factor;
  p.y = pose.y * factor;
  return p;
}

inline double dot(const Pose2d& lhs, const Pose2d& rhs) {
    return lhs.x * rhs.x + lhs.y * rhs.y;
}

inline std::ostream& operator << (std::ostream& ostr, const Pose2d& p){
  ostr << "(" << p.x << " / " << p.y << " / " << (M_PI * 2.0f * p.theta) << "°)";
  return ostr;
}

} // Namespace "lib_path"

#endif // POSE2D_H
