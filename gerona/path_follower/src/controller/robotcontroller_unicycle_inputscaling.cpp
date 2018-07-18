#include <path_follower/controller/robotcontroller_unicycle_inputscaling.h>


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <cslibs_utils/MathHelper.h>
#include <path_follower/utils/pose_tracker.h>
#include <path_follower/utils/visualizer.h>

#include <deque>

#include <limits>
#include <boost/algorithm/clamp.hpp>

#ifdef TEST_OUTPUT
#include <std_msgs/Float64MultiArray.h>
#endif

#include <path_follower/factory/controller_factory.h>

REGISTER_ROBOT_CONTROLLER(RobotController_Unicycle_InputScaling, unicycle_inputscaling, default_collision_avoider);

RobotController_Unicycle_InputScaling::RobotController_Unicycle_InputScaling() :
    RobotController()
{

    double k = params_.k();
    setTuningParameters(k);
    setDirSign(1.f);

    ROS_INFO("Parameters: k=%f\n"
             "vehicle_length=%f\n"
             "goal_tolerance=%f\n"
             "max_angular_velocity=%f",
             params_.k(),
             params_.vehicle_length(),
             params_.goal_tolerance(),
             params_.max_angular_velocity());

#ifdef TEST_OUTPUT
    test_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("test_output", 100);
#endif
}

void RobotController_Unicycle_InputScaling::setTuningParameters(const double k) {
    k1_ = 1. * k * k * k;
    k2_ = 3. * k * k;
}

void RobotController_Unicycle_InputScaling::stopMotion() {

    move_cmd_.setVelocity(0.f);
    move_cmd_.setDirection(0.f);
    move_cmd_.setRotationalVelocity(0.f);

    MoveCommand cmd = move_cmd_;
    publishMoveCommand(cmd);
}

void RobotController_Unicycle_InputScaling::start() {

}

void RobotController_Unicycle_InputScaling::reset() {

    RobotController::reset();
}

void RobotController_Unicycle_InputScaling::setPath(Path::Ptr path) {
    RobotController::setPath(path);
}

RobotController::MoveCommandStatus RobotController_Unicycle_InputScaling::computeMoveCommand(
        MoveCommand* cmd) {

    if(path_interpol.n() <= 2)
        return RobotController::MoveCommandStatus::ERROR;

    const Eigen::Vector3d pose = pose_tracker_->getRobotPose();

    const geometry_msgs::Twist v_meas_twist = pose_tracker_->getVelocity();

    double velocity_measured = dir_sign_ * sqrt(v_meas_twist.linear.x * v_meas_twist.linear.x
                                                + v_meas_twist.linear.y * v_meas_twist.linear.y);

    RobotController::findOrthogonalProjection();
    double d = orth_proj_;

    if(RobotController::isGoalReached(cmd)){
       return RobotController::MoveCommandStatus::REACHED_GOAL;
    }

    // draw a line to the orthogonal projection
    geometry_msgs::Point from, to;
    from.x = pose[0]; from.y = pose[1];
    to.x = path_interpol.p(proj_ind_); to.y = path_interpol.q(proj_ind_);
    visualizer_->drawLine(12341234, from, to, getFixedFrame(), "kinematic", 1, 0, 0, 1, 0.01);


    // theta_e = theta_vehicle - theta_path (orientation error)
    double theta_e = MathHelper::AngleDelta(path_interpol.theta_p(proj_ind_), pose[2]);

    // if dir_sign is negative, we drive backwards and set theta_e to the complementary angle
    if (getDirSign() < 0.) {
        theta_e = MathHelper::NormalizeAngle(M_PI + theta_e);
        d = -d;
    }

    // curvature and first two derivations
    const double c = path_interpol.curvature(proj_ind_);
    const double dc_ds = path_interpol.curvature_prim(proj_ind_);

    // 1 - dc(s)
    const double _1_dc = 1. - d * c;

    // cos, sin, tan of theta_e
    const double cos_theta_e = cos(theta_e);
    const double cos_theta_e_2 = cos_theta_e * cos_theta_e;

    const double sin_theta_e = sin(theta_e);
    const double sin_theta_e_2 = sin_theta_e * sin_theta_e;

    const double tan_theta_e = tan(theta_e);

    //	const double x1 = s;
    const double x2 = _1_dc * tan_theta_e;

    const double x3 = d;


    // u1, u2

    const double u1 = velocity_measured * cos_theta_e / _1_dc;
    const double u2 =
            - k1_ * u1 * x3
            - k2_ * fabs(u1) * x2;


    // longitudinal velocity
    const double v1 = velocity_;

    // angle velocity
    double v2 = u2 * cos_theta_e_2 / _1_dc +
            u1 * (c * (1 + sin_theta_e_2) + d * dc_ds * (sin_theta_e * cos_theta_e)/_1_dc);

    v2 = boost::algorithm::clamp(v2, -params_.max_angular_velocity(), params_.max_angular_velocity());

    double exp_factor = RobotController::exponentialSpeedControl();
    move_cmd_.setRotationalVelocity(v2);
    move_cmd_.setVelocity(getDirSign() * v1 * exp_factor);
    *cmd = move_cmd_;


#ifdef TEST_OUTPUT
    publishTestOutput(proj_ind_, d, theta_e, velocity_measured);
#endif


    *cmd = move_cmd_;

    return RobotController::MoveCommandStatus::OKAY;

}

void RobotController_Unicycle_InputScaling::publishMoveCommand(
        const MoveCommand& cmd) const {

    geometry_msgs::Twist msg;
    msg.linear.x  = cmd.getVelocity();
    msg.linear.y  = 0;
    msg.angular.z = cmd.getRotationalVelocity();

    cmd_pub_.publish(msg);
}


#ifdef TEST_OUTPUT
void RobotController_Unicycle_InputScaling::publishTestOutput(const unsigned int waypoint, const double d,
                                                              const double theta_e,
                                                              const double v) const {
    std_msgs::Float64MultiArray msg;

    msg.data.push_back((double) waypoint);
    msg.data.push_back(d);
    msg.data.push_back(theta_e);
    msg.data.push_back(v);

    test_pub_.publish(msg);
}
#endif

