/**
   @author Andreas Beck-Greinwald
   @date   2012-01-15
   @file   CentroidRadiusGoalRegion.cpp
*/ 

#include <math.h>
#include <cslibs_path_planning/sampling/CentroidRadiusGoalRegion.h>

using namespace lib_path;
using namespace Eigen;

CentroidRadiusGoalRegion::CentroidRadiusGoalRegion (const Point2d &src, const Point2d &center, double radius, double angle_rad)
  : src_ (src), center_ (center), radius_ (radius), angle_rad_ (angle_rad), counter_ (0), samples_num_ (0)
{
	// No init -> done by sampling planner
}


CentroidRadiusGoalRegion::~CentroidRadiusGoalRegion ()
{
  // nothing to do
}


void CentroidRadiusGoalRegion::init (unsigned samples_num)
{
  counter_ = 0;
  samples_num_ = samples_num;

  // Direct approach angle of robot (src) to goal (center)
  theta_ = MathHelper::Angle (Vector2d (center_.x - src_.x, center_.y - src_.y));

  angle_rad_ = fabs (angle_rad_);
  if (angle_rad_ < 1e-6 || samples_num_ <= 1) {
	  samples_num_ = 1;
	  theta_step_ = 0.0;
	  return;
  }

  theta_ -= 0.5 * angle_rad_;
  theta_step_ = angle_rad_ / (samples_num_-1);
}


bool CentroidRadiusGoalRegion::getNextGoal (Pose2d &goal, double& gain)
{
  if (counter_ >= samples_num_)
    return false;

  goal.x = center_.x - radius_ * cos (theta_);
  goal.y = center_.y - radius_ * sin (theta_);
  goal.theta = theta_;

  counter_++;
  theta_ = MathHelper::AngleClamp (theta_ + theta_step_);

  // Direct approach 1.0 -> approach from max angle 2.0
  int32_t diff = (int32_t)(counter_ - (samples_num_/2));
  gain = 1.0 + 2.0*abs (diff) / samples_num_;

  return true;
}

