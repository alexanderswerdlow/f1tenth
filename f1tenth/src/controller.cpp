#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <race/drive_values.h>
#include <race/drive_flags.h>
#include <std_msgs/Bool.h>
#include <math.h>

ros::Time prevTime;
ros::Publisher pwm_pub, flag_pub;
double desiredThrottle, desiredSteering, prevError, wheelbase, prevOutput, maxVel;
double kP = 0.01;
double kI = 0.0;
double kD = kP * 10;
nav_msgs::Odometry lastOdom;
nav_msgs::Odometry *p = &lastOdom;
bool firstRun;
inline bool sameSign(double a, double b) {
  return a * b >= 0.0f;
}

double convert_trans_rot_vel_to_steering_angle(double v, double omega, double wheelbase) {
  if (omega == 0 || v == 0) {
    return 0;
  }
  double radius = v / omega;
  return atan(wheelbase / radius);
}
double arduino_map(double x, double in_min, double in_max, double out_min, double out_max) {
  double val = x;
  if (val < in_min) {
    val = in_min;
  } else if (val > in_max) {
    val = in_max;
  }
  return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void updateOdom(nav_msgs::Odometry data) {
  lastOdom = data;
  if (firstRun) {
    race::drive_flags temp;
    flag_pub.publish(temp);
    firstRun = false;
  }
}

void updateCmd(geometry_msgs::Twist data) {
  desiredThrottle = data.linear.x;
  desiredSteering = convert_trans_rot_vel_to_steering_angle(data.linear.x, data.angular.z, wheelbase);
  double output = 0;
  if (p != NULL) {
    double compVel = sqrt(pow(lastOdom.twist.twist.linear.x, 2) + pow(lastOdom.twist.twist.linear.y, 2));
    ros::Time currentTime = ros::Time::now();
    double dt = currentTime.toSec() - prevTime.toSec();
    double err = desiredThrottle - compVel;
    double kpOut = (err * kP);
    double kDOut = fabs((err - prevError)) > 3.0 ? 0.0 : ((err - prevError) / (dt));
    output = fabs(err) > 0.05 ? desiredThrottle + kpOut + kDOut : desiredThrottle;
    output = sameSign(output, desiredThrottle) ? output : 0.0;
    prevTime = currentTime;
    prevError = err;
    prevOutput = output;
  } else {
    output = 0;
  }

  double pwm1 = arduino_map(desiredThrottle, -1.7, 1.7, 6554, 13108);
  double pwm2 = arduino_map(-tfDegrees(desiredSteering), -25, 25, 6554, 12241);
  race::drive_values msg;
  msg.pwm_drive = pwm1;
  msg.pwm_angle = pwm2;
  pwm_pub.publish(msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "controller");
  ros::NodeHandle n;

  pwm_pub = n.advertise<race::drive_values>("drive_pwm", 100);
  flag_pub = n.advertise<race::drive_flags>("driveFlags", 100);

  prevTime = ros::Time::now();
  desiredThrottle = desiredSteering = prevError = prevOutput = maxVel = 0;
  p = NULL;
  firstRun = true;

  n.param("wheelabase", wheelbase, 0.32385);

  ros::Subscriber cmdSub = n.subscribe("cmd_vel", 100, updateCmd);
  ros::Subscriber odomSub = n.subscribe("odometry/filtered", 100, updateOdom);

  ros::spin();

  return 0;
}