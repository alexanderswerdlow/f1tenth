#ifndef HIGH_SPEED_LOCAL_PLANNER_H
#define HIGH_SPEED_LOCAL_PLANNER_H

/// PROJECT
#include <path_follower/local_planner/abstract_local_planner.h>

/// SYSTEM
#include <ros/time.h>

class HighSpeedLocalPlanner : public AbstractLocalPlanner
{
public:
    HighSpeedLocalPlanner();

    virtual Path::Ptr updateLocalPath() override;

    virtual void setGlobalPath(Path::Ptr path) override;
private:
    bool transform2Odo(ros::Time& now);

    void printSCTimeUsage();

    virtual void printNodeUsage(std::size_t& nnodes) const = 0;
    virtual void printVelocity() = 0;
    virtual void printLevelReached() const = 0;
    virtual bool algo(Eigen::Vector3d& pose, SubPath& local_wps,
                     std::size_t& nnodes) = 0;
protected:
    SubPath waypoints, wlp_;
    SubPath waypoints_map;

    bool close_to_goal;

    std::vector<SubPath> all_local_paths_;

    ros::Time last_update_;
};

#endif // HIGH_SPEED_LOCAL_PLANNER_H
