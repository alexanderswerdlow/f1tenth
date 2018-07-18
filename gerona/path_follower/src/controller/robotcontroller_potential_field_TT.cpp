// HEADER
#include <path_follower/controller/robotcontroller_potential_field_TT.h>

// THIRD PARTY
#include <tf/tf.h>

// PROJECT
#include <path_follower/utils/pose_tracker.h>
#include <path_follower/utils/obstacle_cloud.h>
#include <path_follower/parameters/path_follower_parameters.h>
#include <path_follower/collision_avoidance/collision_avoider.h>
#include <cslibs_utils/MathHelper.h>

// SYSTEM
#include <boost/algorithm/clamp.hpp>

#include <path_follower/factory/controller_factory.h>

REGISTER_ROBOT_CONTROLLER(RobotController_Potential_Field_TT, potential_field_TT, default_collision_avoider);

RobotController_Potential_Field_TT::RobotController_Potential_Field_TT():
    RobotController_Potential_Field()
{
}

void RobotController_Potential_Field_TT::setGoalPosition()
{

    Eigen::Vector3d current_pose = pose_tracker_->getRobotPose();
    double x_meas = current_pose[0];
    double y_meas = current_pose[1];
    double theta_meas = current_pose[2];

    ///PERSON FOLLOWING
    double s_diff = std::numeric_limits<double>::max();

    // desired distance behind the person in path coordinates
    double s_dist = 2.0;
    tf::Point goal (0,0,0);
    for(int i = proj_ind_; i < ((int)path_interpol.n())-1 ; i++){
        if(fabs(fabs(path_interpol.s(path_interpol.n()-1) - path_interpol.s(i)) - s_dist) < s_diff){
            s_diff = fabs(fabs(path_interpol.s(path_interpol.n()-1) - path_interpol.s(i)) - s_dist);
            tf::Point goal_tmp(path_interpol.p(i), path_interpol.q(i), 0.0);
            goal = goal_tmp;
        }
        if(fabs(fabs(path_interpol.s(path_interpol.n()-1) - path_interpol.s(proj_ind_)) - s_dist) < 1e-1){
            vn_ = 0.0;
        }
    }


    mGoalPosX = goal.getX();
    mGoalPosY = goal.getY();

    double dx = mGoalPosX - x_meas;
    double dy = mGoalPosY - y_meas;

    FAttX = opt_.kAtt()*(dx*cos(-theta_meas) - dy*sin(-theta_meas));
    FAttY = opt_.kAtt()*(dx*sin(-theta_meas) + dy*cos(-theta_meas));

}


