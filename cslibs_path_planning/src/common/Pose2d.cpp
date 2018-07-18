/*
 * Pose2d.cpp
 *
 *  Created on: Apr 4, 2011
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */


#include <cslibs_utils/MathHelper.h>

#include <cslibs_path_planning/common/Pose2d.h>

using namespace lib_path;
using namespace std;

Pose2d::Pose2d()
  : x(0), y(0), theta(0)
{
}

Pose2d::Pose2d(double x, double y, double theta)
  : x(x), y(y), theta(theta)
{
}

double Pose2d::distance_to(const Pose2d &rhs) const
{
    double dx = x-rhs.x;
    double dy = y-rhs.y;
    return sqrt(dx*dx + dy*dy);
}

double Pose2d::distance_to_origin() const
{
    return sqrt(x*x + y*y);
}

bool Pose2d::isEqual( const Pose2d &p,
                      const double dist_eps,
                      const double theta_eps ) const
{
    return distance_to( p ) <= dist_eps
            && fabs( MathHelper::AngleDelta( theta, p.theta )) <= theta_eps;
}
