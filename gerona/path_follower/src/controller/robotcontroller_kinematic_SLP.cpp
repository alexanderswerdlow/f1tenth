// HEADER
#include <path_follower/controller/robotcontroller_kinematic_SLP.h>

// THIRD PARTY
#include <visualization_msgs/MarkerArray.h>

// PROJECT
#include <path_follower/parameters/path_follower_parameters.h>
#include <path_follower/utils/cubic_spline_interpolation.h>
#include <cslibs_utils/MathHelper.h>
#include <path_follower/utils/pose_tracker.h>
#include <path_follower/utils/visualizer.h>

// SYSTEM
#include <cmath>
#include <deque>
#include <boost/algorithm/clamp.hpp>


#include <path_follower/factory/controller_factory.h>

REGISTER_ROBOT_CONTROLLER(RobotController_Kinematic_SLP, kinematic_SLP, default_collision_avoider);

using namespace Eigen;


RobotController_Kinematic_SLP::RobotController_Kinematic_SLP():
    RobotController(),
    cmd_(this),
    vn_(0),
    delta_(0),
    Ts_(0.02),
    ind_(0),
    xe_(0),
    ye_(0)
{

}

void RobotController_Kinematic_SLP::stopMotion()
{

    cmd_.speed = 0;
    cmd_.direction_angle = 0;
    cmd_.rotation = 0;

    MoveCommand mcmd = cmd_;
    publishMoveCommand(mcmd);
}

void RobotController_Kinematic_SLP::initialize()
{
    RobotController::initialize();

    //reset the index of the current point on the path
    ind_ = 0;

    // desired velocity
    vn_ = std::min(PathFollowerParameters::getInstance()->max_velocity(), velocity_);
    ROS_DEBUG_STREAM("velocity_: " << velocity_ << ", vn: " << vn_);
}

void RobotController_Kinematic_SLP::start()
{

}

void RobotController_Kinematic_SLP::reset()
{
    RobotController::reset();
}

void RobotController_Kinematic_SLP::setPath(Path::Ptr path)
{
    RobotController::setPath(path);
}

RobotController::MoveCommandStatus RobotController_Kinematic_SLP::computeMoveCommand(MoveCommand *cmd)
{
    // omni drive can rotate.
    *cmd = MoveCommand(true);

    if(path_interpol.n() < 2) {
        ROS_ERROR("[Line] path is too short (N = %d)", (int) path_interpol.n());

        stopMotion();
        return MoveCommandStatus::REACHED_GOAL;
    }


    /// get the pose as pose(0) = x, pose(1) = y, pose(2) = theta
    Eigen::Vector3d current_pose = pose_tracker_->getRobotPose();

    double x_meas = current_pose[0];
    double y_meas = current_pose[1];
    double theta_meas = current_pose[2];
    ///***///


    RobotController::findOrthogonalProjection();

    if(RobotController::isGoalReached(cmd)){
       return RobotController::MoveCommandStatus::REACHED_GOAL;
    }


    ///Compute the control for the current point on the path

    //robot direction angle in path coordinates
    double theta_e = MathHelper::AngleDelta(path_interpol.theta_p(ind_), theta_meas);

    //robot position vector module
    double r = hypot(x_meas - path_interpol.p(ind_), y_meas - path_interpol.q(ind_));

    //robot position vector angle in world coordinates
    double theta_r = atan2(y_meas - path_interpol.q(ind_), x_meas - path_interpol.p(ind_));

    //robot position vector angle in path coordinates
    double delta_theta = MathHelper::AngleDelta(path_interpol.theta_p(ind_), theta_r);

    //current robot position in path coordinates
    xe_ = r * cos(delta_theta);
    ye_ = r * sin(delta_theta);

    ///***///


    ///Check the driving direction, and set the complementary angle in path coordinates, if driving backwards

    if (getDirSign() < 0.0) {
        theta_e = MathHelper::NormalizeAngle(M_PI + theta_e);
        ROS_WARN_THROTTLE(1, "Driving backwards...");
    }

    ///***///


    ///Compute the delta_ and its derivative

    double delta_old = delta_;

    delta_ = MathHelper::AngleClamp(-getDirSign()*opt_.theta_a()*tanh(ye_));

    double delta_prim = (delta_ - delta_old)/Ts_;
    ///***///


    ///Lyapunov-curvature speed control

    //Lyapunov function as a measure of the path following error
    double v = vn_;
    double V1 = 0.5*(std::pow(xe_,2) + std::pow(ye_,2)) + (0.5/opt_.gamma())*std::pow((theta_e - delta_),2);

    //use v/2 as the minimum speed, and allow larger values when the error is small
    if (V1 >= opt_.epsilon()) v = 0.5*v;

    else if (V1 < opt_.epsilon()) v = v/(1 + opt_.b()*std::abs(path_interpol.curvature(ind_)));

    ///***///


    ///Compute the next point on the path

    double s_old = path_interpol.s_new();

    //calculate the speed of the "virtual vehicle"
    double s_prim_tmp = v * cos(theta_e) + opt_.k1() * xe_;
    path_interpol.set_s_prim(s_prim_tmp > 0 ? s_prim_tmp : 0);

    //approximate the first derivative and calculate the next point
    double s_temp = Ts_*path_interpol.s_prim() + s_old;
    path_interpol.set_s_new(s_temp > 0 ? s_temp : 0);

    ///***///


    ///Direction control

    cmd_.direction_angle = 0;

    //omega_m = theta_e_prim + curv*s_prim
    double omega_m = delta_prim - opt_.gamma()*ye_*v*(sin(theta_e) - sin(delta_))
            /(theta_e - delta_) - opt_.k2()*(theta_e - delta_) + path_interpol.curvature(ind_)*path_interpol.s_prim();


    omega_m = boost::algorithm::clamp(omega_m, -opt_.max_angular_velocity(), opt_.max_angular_velocity());
    cmd_.rotation = omega_m;

    ///***///

    ///Speed control

    double exp_factor = RobotController::exponentialSpeedControl();
    v = v * exp_factor;

    cmd_.speed = getDirSign()*std::max((double)PathFollowerParameters::getInstance()->min_velocity(), fabs(v));

    ///***///



    ///plot the moving reference frame together with position vector and error components

    if (visualizer_->MarrayhasSubscriber()) {
        visualizer_->drawFrenetSerretFrame(getFixedFrame(), 0, current_pose, xe_, ye_, path_interpol.p(ind_),
                                           path_interpol.q(ind_), path_interpol.theta_p(ind_));
    }

    ///***///


    ///Compute the index of the new point

    double s_diff = std::numeric_limits<double>::max();
    uint old_ind = ind_;

    for (unsigned int i = old_ind; i < path_interpol.n(); i++){

        double s_diff_curr = std::abs(path_interpol.s_new() - path_interpol.s(i));

        if(s_diff_curr < s_diff){

            s_diff = s_diff_curr;
            ind_ = i;

        }

    }

    if(old_ind != ind_) {
        path_->fireNextWaypointCallback();
    }

    ///***///


    if (visualizer_->hasSubscriber()) {
        visualizer_->drawSteeringArrow(pose_tracker_->getFixedFrameId(), 1, pose_tracker_->getRobotPoseMsg(), cmd_.direction_angle, 0.2, 1.0, 0.2);
    }

    ///***///

    *cmd = cmd_;

    return MoveCommandStatus::OKAY;
}

void RobotController_Kinematic_SLP::publishMoveCommand(const MoveCommand &cmd) const
{
    geometry_msgs::Twist msg;
    msg.linear.x  = cmd.getVelocity();
    msg.linear.y  = 0;
    msg.angular.z = cmd.getRotationalVelocity();

    cmd_pub_.publish(msg);
}
