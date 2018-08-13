#include "ros/ros.h"
#include <filters/filter_base.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>
#include <iostream>   // std::cout
#include <string>     // std::string, std::to_string

ros::Publisher pub;

double lower_angle_ = 0;
double upper_angle_ = 0;
bool run = false;
sensor_msgs::LaserScan lastScan;

sensor_msgs::LaserScan update(const sensor_msgs::LaserScan &input_scan) {
  sensor_msgs::LaserScan filtered_scan;
  filtered_scan = input_scan; //copy entire message

  double current_angle = input_scan.angle_min;
  unsigned int count = 0;
  //loop through the scan and remove ranges at angles between lower_angle_ and upper_angle_
  for (unsigned int i = 0; i < input_scan.ranges.size(); ++i) {
	if ((current_angle > lower_angle_) && (current_angle < upper_angle_)) {
	  filtered_scan.ranges[i] = input_scan.range_max + 1.0;
	  if (i < filtered_scan.intensities.size()) {
		filtered_scan.intensities[i] = 0.0;
	  }
	  count++;
	}
	current_angle += input_scan.angle_increment;
  }

  ROS_DEBUG("Filtered out %u points from the laser scan.", count);

  return filtered_scan;

}

bool compare(double a, double b) {
  return fabs(a - b) < 0.001;
}

void updateCmd(sensor_msgs::LaserScan data) {
  sensor_msgs::LaserScan tempScan = data;
  if(!run){
	run = true;
	lastScan = tempScan;
  }
  double current_angle = tempScan.angle_min;
  bool prev = false;
  double minAngle = 0;

  for (unsigned int i = 0; i < tempScan.ranges.size(); ++i) {

	if (compare(lastScan.ranges[i], tempScan.ranges[i])) {

	  if(!prev){
		prev = true;
		minAngle = current_angle;
	  }

	} else if(prev){
	  prev = false;
	  if(fabs(minAngle - (current_angle - tempScan.angle_increment)) > 0.1){
		std::cout << minAngle << "---" << current_angle - tempScan.angle_increment << std::endl;
	  }
	}
	current_angle += tempScan.angle_increment;
  }
  lastScan = tempScan;
}

int main(int argc, char **argv) {



  ros::init(argc, argv, "controller");

  ros::NodeHandle n;
  pub = n.advertise<sensor_msgs::LaserScan>("laser_scan", 1000);

  ros::Rate loop_rate(100);

  ros::Subscriber cmdSub = n.subscribe("scan", 1000, updateCmd);

  ros::spin();

  return 0;
}