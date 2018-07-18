#include <path_follower/controller/robotcontroller_ackermann_inputscaling.h>


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <path_follower/utils/pose_tracker.h>
#include <path_follower/utils/visualizer.h>
#include <cslibs_utils/MathHelper.h>
#include <deque>
#include <limits>
#include <boost/algorithm/clamp.hpp>
#include <time.h>
#include <path_follower/factory/controller_factory.h>

REGISTER_ROBOT_CONTROLLER(RobotController_Ackermann_Inputscaling, ackermann_inputscaling, ackermann);

RobotController_Ackermann_Inputscaling::RobotController_Ackermann_Inputscaling() :
    RobotController(),
    phi_(0.),
	v1_(0.), v2_(0.)
{

	const double k = params_.k_forward();
	setTuningParameters(k);

	ROS_INFO("Parameters: k_forward=%f, k_backward=%f\n"
				"factor_k1=%f, k2=%f, k3=%f\n"
				"vehicle_length=%f\n"
				"factor_steering_angle=%f\n"
				"goal_tolerance=%f\nmax_steering_angle=%f",
				params_.k_forward(), params_.k_backward(),
				params_.factor_k1(), params_.factor_k2(), params_.factor_k3(),
				params_.vehicle_length(),
				params_.factor_steering_angle(),
				params_.goal_tolerance(), params_.max_steering_angle());

}

void RobotController_Ackermann_Inputscaling::setTuningParameters(const double k) {
	k1_ = params_.factor_k1() * k * k * k;
	k2_ = params_.factor_k2() * k * k;
	k3_ = params_.factor_k3() * k;
}

void RobotController_Ackermann_Inputscaling::stopMotion() {

	move_cmd_.setVelocity(0.f);
	move_cmd_.setDirection(0.f);

	phi_ = 0.;

	MoveCommand cmd = move_cmd_;
	publishMoveCommand(cmd);
}

void RobotController_Ackermann_Inputscaling::start() {

}

void RobotController_Ackermann_Inputscaling::reset() {
	old_time_ = ros::Time::now();

	v1_ = v2_ = 0.;

	s_prim_ = 0.001; // TODO: good starting value


    RobotController::reset();
}

void RobotController_Ackermann_Inputscaling::setPath(Path::Ptr path)
{
    RobotController::setPath(path);
}

RobotController::MoveCommandStatus RobotController_Ackermann_Inputscaling::computeMoveCommand(
        MoveCommand* cmd)
{
	if(path_interpol.n() <= 2)
		return RobotController::MoveCommandStatus::ERROR;

    const Eigen::Vector3d pose = pose_tracker_->getRobotPose();
    const geometry_msgs::Twist velocity_measured = pose_tracker_->getVelocity();

	ROS_DEBUG("velocity_measured: x=%f, y=%f, z=%f", velocity_measured.linear.x,
				velocity_measured.linear.y, velocity_measured.linear.z);

    RobotController::findOrthogonalProjection();
    double d = orth_proj_;

    if(RobotController::isGoalReached(cmd)){
       return RobotController::MoveCommandStatus::REACHED_GOAL;
    }

	// draw a line to the orthogonal projection
	geometry_msgs::Point from, to;
    from.x = pose[0]; from.y = pose[1];
    to.x = path_interpol.p(proj_ind_); to.y = path_interpol.q(proj_ind_);
    visualizer_->drawLine(12341234, from, to, getFixedFrame(), "Inputscaling", 1, 0, 0, 1, 0.01);

	// theta_e = theta_vehicle - theta_path (orientation error)
    double theta_e = MathHelper::AngleDelta(path_interpol.theta_p(proj_ind_), pose[2]);

	// if dir_sign is negative we drive backwards and set theta_e to the complementary angle
    if (getDirSign() < 0.) {
        d = -d;
		setTuningParameters(params_.k_backward());
        theta_e = theta_e > 0.? M_PI - theta_e : -M_PI - theta_e;
	} else {
		setTuningParameters(params_.k_forward());
	}

	// curvature and first two derivations
    const double c = path_interpol.curvature(proj_ind_);
    const double c_prim = path_interpol.curvature_prim(proj_ind_);
    const double c_sek = path_interpol.curvature_sek(proj_ind_);

	// 1 - dc(s)
	const double _1_dc = 1. - d * c;
	const double _1_dc_2 = _1_dc * _1_dc;


	// cos, sin, tan of theta_e and phi
	const double cos_theta_e = cos(theta_e);
	const double cos_theta_e_2 = cos_theta_e * cos_theta_e;
	const double cos_theta_e_3 = cos_theta_e_2 * cos_theta_e;

	const double sin_theta_e = sin(theta_e);
	const double sin_theta_e_2 = sin_theta_e * sin_theta_e;

	const double tan_theta_e = tan(theta_e);
	const double tan_theta_e_2 = tan_theta_e * tan_theta_e;

	const double tan_phi = tan(phi_);
	const double tan_phi_2 = tan_phi * tan_phi;


	const double time_passed = (ros::Time::now() - old_time_).toSec();
	old_time_ = ros::Time::now();

	v1_ = abs(velocity_measured.linear.x);

	s_prim_ = cos_theta_e / _1_dc;

	const double dd_ds = sin_theta_e * v1_ / s_prim_;

    //theta_e_prim_ = (theta_e - old_theta_e_) * delta_s_inverse;
	const double dtheta_e_ds = ((tan_phi / params_.vehicle_length() - c * cos_theta_e / _1_dc) * v1_)
			/ s_prim_;

    // follows from: phi_prim = phi / t, s_prim = s / t, v2 = phi / t
	const double dphi_ds = v2_ / s_prim_;

	ROS_DEBUG("s_prim=%f, delta_s=%f", s_prim_, s_prim_ * time_passed);
	ROS_DEBUG("d'=%f, theta_e'=%f, phi'=%f", dd_ds, dtheta_e_ds, dphi_ds);

	//
	// actual controller formulas begin here
	//

	// x1 - x4
	//	const double x1 = s;
	const double x2 = -c_prim * d * tan_theta_e
			- c * _1_dc * (1. + sin_theta_e_2) / cos_theta_e_2
			+ _1_dc_2 * tan_phi / (params_.vehicle_length() * cos_theta_e_3);

	const double x3 = _1_dc * tan_theta_e;
	const double x4 = d;

	// u1, u2
	// u1 is taken from "Feedback control for a path following robotic car" by Mellodge,
	// p. 108 (u1_actual)

	// TODO: use measured velocity
	const double u1 = velocity_ * cos_theta_e / _1_dc;
	const double u2 =
            - k1_ * u1 * x4
            - k2_ * u1 * x3
            - k3_ * u1 * x2;

	// derivations of x2 (for alpha1)
	const double dx2_dd = -c_prim * tan_theta_e
			+ c * c * (1 + sin_theta_e_2) / cos_theta_e_2
			- 2. * _1_dc * c * tan_phi / (params_.vehicle_length() * cos_theta_e_3);

	const double dx2_dtheta_p = -c_prim * (tan_theta_e_2 + 1.)
			- 4. * c * _1_dc * tan_theta_e / cos_theta_e_2
            + 3. * _1_dc_2 * tan_phi * tan_theta_e / (params_.vehicle_length() * cos_theta_e_3);

	const double dx2_ds =
			-tan_theta_e * (c_sek * d + c_prim * dd_ds)
			- c_prim * d * dtheta_e_ds * (1. + tan_theta_e_2)
			+ ((1. + sin_theta_e_2) / cos_theta_e_2) * (c_prim * _1_dc + c * (dd_ds * c + d * c_prim))
			- 4. * c * _1_dc * tan_theta_e / cos_theta_e_2
			+ (cos_theta_e * _1_dc * (-2. * (dd_ds * c + d * c_prim) * tan_phi
											  +_1_dc * (1. + tan_phi_2) * dphi_ds)
				- 3. * dtheta_e_ds * sin_theta_e * _1_dc_2 * tan_phi)
			/ (params_.vehicle_length() * pow(cos_theta_e_2, 2)); // OK

	// alpha1
	const double alpha1 =
			dx2_ds
			+ dx2_dd * _1_dc * tan_theta_e
			+ dx2_dtheta_p * (tan_phi * _1_dc / (params_.vehicle_length() * cos_theta_e) - c);

	// alpha2
	const double alpha2 =
			params_.vehicle_length() * cos_theta_e_3 * pow(cos(phi_), 2) / _1_dc_2;


	// longitudinal velocity
    v1_ = velocity_;

	// steering angle velocity
	v2_ = alpha2 * (u2 - alpha1 * u1);

	// limit steering angle velocity
	v2_ = boost::algorithm::clamp(v2_, -params_.max_steering_angle_speed(), params_.max_steering_angle_speed());

	// update delta according to the time that has passed since the last update
	phi_ += v2_ * time_passed;

	// also limit the steering angle
	phi_ = boost::algorithm::clamp(phi_, -params_.max_steering_angle(), params_.max_steering_angle());

	ROS_DEBUG("d=%f, thetaP=%f, c=%f, c'=%f, c''=%f", d, theta_e, c, c_prim, c_sek);
	ROS_DEBUG("d'=%f, thetaP'=%f", dd_ds, dtheta_e_ds);
	ROS_DEBUG("1 - dc(s)=%f", _1_dc);
	ROS_DEBUG("dx2dd=%f, dx2dthetaP=%f, dx2ds=%f", dx2_dd, dx2_dtheta_p, dx2_ds);
	ROS_DEBUG("alpha1=%f, alpha2=%f, u1=%f, u2=%f", alpha1, alpha2, u1, u2);
	ROS_DEBUG("Time passed: %fs, command: v1=%f, v2=%f, phi_=%f",
				 time_passed, v1_, v2_, phi_);

	// This is the accurate steering angle for 4 wheel steering (TODO: wrong!!!)
	const float delta = (float) asin(params_.factor_steering_angle() * sin(phi_));

    double exp_factor = RobotController::exponentialSpeedControl();
	move_cmd_.setDirection(delta);
    move_cmd_.setVelocity(getDirSign() * (float) v1_ * exp_factor);
	*cmd = move_cmd_;

	return RobotController::MoveCommandStatus::OKAY;
}

void RobotController_Ackermann_Inputscaling::publishMoveCommand(
		const MoveCommand& cmd) const {

	geometry_msgs::Twist msg;
	msg.linear.x  = cmd.getVelocity();
	msg.linear.y  = 0;
	msg.angular.z = cmd.getDirectionAngle();

	cmd_pub_.publish(msg);
}
