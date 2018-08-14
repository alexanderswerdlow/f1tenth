#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <race/drive_param.h>
#include <math.h>

ros::Time prevTime;
ros::Publisher pub;
double desiredThrottle, desiredSteering, prevError, wheelbase, prevOutput, maxVel;
double kP = 0.05;
double kI = 0.0;
double kD = 0.0;

inline bool sameSign(double a, double b) {
  return a * b >= 0.0f;
}
double getYawFromPose(const geometry_msgs::Pose &carPose) {
  double x = carPose.orientation.x;
  double y = carPose.orientation.y;
  double z = carPose.orientation.z;
  double w = carPose.orientation.w;

  double tmp, yaw;
  tf::Quaternion q(x, y, z, w);
  tf::Matrix3x3 quaternion(q);
  quaternion.getRPY(tmp, tmp, yaw);

  return yaw;
}

double convert_trans_rot_vel_to_steering_angle(double v, double omega, double wheelbase) {
  if (omega == 0 || v == 0) {
	return 0;
  }
  double radius = v / omega;
  return atan(wheelbase / radius);
}
void updateOdom(nav_msgs::Odometry data) {
  //maxVel = data.twist.twist.linear.x > maxVel ? data.twist.twist.linear.x : maxVel;
  //std::cout << maxVel << std::endl;
  double compVel = sqrt(pow(data.twist.twist.linear.x, 2) + pow(data.twist.twist.linear.y, 2));
  ros::Time currentTime = ros::Time::now();
  double dt = currentTime.toSec() - prevTime.toSec();
  double err = desiredThrottle - compVel;
  double kpOut = (err * kP);
  double kDOut = fabs((err - prevError)) > 1.0 ? 0.0 : ((err - prevError) / (dt));
  double output = desiredThrottle + kpOut + kDOut;
  output = output > 3.0 ? 3.0 : output;
  output = output < -3.0 ?-3.0 : output;
  output = sameSign(output, desiredThrottle) ? output : 0.0;
  prevTime = currentTime;
  prevError = err;
  prevOutput = output;
  race::drive_param msg;
  msg.velocity = (float) (desiredThrottle);
  msg.angle = (float) tfDegrees(desiredSteering);
  pub.publish(msg);
}

void updateCmd(geometry_msgs::Twist data) {
  desiredThrottle = data.linear.x;
  desiredSteering = convert_trans_rot_vel_to_steering_angle(data.linear.x, data.angular.z, wheelbase);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "controller");

  ros::NodeHandle n;
  pub = n.advertise<race::drive_param>("drive_parameters", 1000);
  prevTime = ros::Time::now();
  desiredThrottle = desiredSteering = prevError = prevOutput = maxVel = 0;
  n.param("wheelabase", wheelbase, 0.32);

  ros::Rate loop_rate(100);

  ros::Subscriber cmdSub = n.subscribe("cmd_vel", 100, updateCmd);
  ros::Subscriber odomSub = n.subscribe("odometry/filtered", 100, updateOdom);

  ros::spin();

  return 0;
}