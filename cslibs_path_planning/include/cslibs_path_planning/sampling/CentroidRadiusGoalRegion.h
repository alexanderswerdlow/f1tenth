/**
   @author Andreas Beck-Greinwald
   @date   2012-01-15
   @file   CentroidRadiusGoalRegion.h
*/ 

#ifndef CENTROIDRADIUSGOALREGION_H
#define CENTROIDRADIUSGOALREGION_H
#include "../common/Point2d.h"
#include "GoalRegion.h"

namespace lib_path {

class CentroidRadiusGoalRegion : public GoalRegion
{
public:

  CentroidRadiusGoalRegion (const Point2d &src, const Point2d &center, double radius, double angle_rad);
  virtual ~CentroidRadiusGoalRegion ();

  virtual void init (unsigned samples_num);
  virtual bool getNextGoal (Pose2d &goal, double& gain);

private:

  Point2d src_;
  Point2d center_;

  double radius_;
  double angle_rad_;

  double theta_;
  double theta_step_;

  unsigned counter_;
  unsigned samples_num_;
};

} // Namespace lib_path

#endif // CENTROIDRADIUSGOALREGION_H

