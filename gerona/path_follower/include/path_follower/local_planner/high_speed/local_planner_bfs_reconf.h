#ifndef LOCAL_PLANNER_BFS_RECONF_H
#define LOCAL_PLANNER_BFS_RECONF_H

/// PROJECT
#include <path_follower/local_planner/high_speed/local_planner_bfs.h>
#include <path_follower/local_planner/high_speed/local_planner_reconf.h>

class LocalPlannerBFSReconf : public LocalPlannerBFS, public LocalPlannerReconf
{
public:
    LocalPlannerBFSReconf();
private:
    virtual void evaluate(double& current_p, LNode*& succ, double& dis2last) override;
};

#endif // LOCAL_PLANNER_BFS_RECONF_H
