/// HEADER
#include <path_follower/local_planner/high_speed/local_planner_bfs_reconf.h>

/// PROJECT
#include <path_follower/factory/local_planner_factory.h>

REGISTER_LOCAL_PLANNER(LocalPlannerBFSReconf, HS_BFSR);


LocalPlannerBFSReconf::LocalPlannerBFSReconf()
{

}

void LocalPlannerBFSReconf::evaluate(double& current_p, LNode*& succ, double& dis2last){
    (void) current_p;
    (void) succ;
    (void) dis2last;
}
