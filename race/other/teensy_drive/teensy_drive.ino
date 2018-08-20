#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <race/chatter_values.h>
#include <race/drive_values.h>
#include <race/drive_flags.h>
#inclide "math.h"
ros::NodeHandle nh;
race::chatter_values debug_msg; // creater a ROS Publisher called chatter of type debug_msg
ros::Publisher chatter("chatter", &debug_msg);

boolean eStopFlag = false;
boolean controlOverrideFlag = false;
int pwm_center_value = 9830;
int steering_trim = 433;
int pwm_center_steering_value = pwm_center_value - steering_trim;
int constrained_pwm_lowerlimit = 9230;
int constrained_pwm_upperlimit = 10430;
int pwm_lowerlimit = 6554;
int pwm_upperlimit = 13108;
int steeringPWMInput = 1500;
int steeringPWMPrevTime = 0;
int throttlePWMInput = 1434;
int throttlePWMPrevTime = 0;

long lastDriveCommandTime = 0;
long lastDebugMsg = 0;
int throttle = pwm_center_value;
int steering = pwm_center_value - steering_trim;
int escPin = 5;
int servoPin = 6;
int escControl = 2;
int servoControl = 3;

// Pin 5 is connected to the ESC..dive motor
// Pin 6 is connected to the steering servo.
// Pin 2 is throttle input from the Reciever
// Pin 3 is the steering input from the Reciever

void set(int esc, int servo) {
  int cappedEsc = esc;
  if (cappedEsc < constrained_pwm_lowerlimit) {
	cappedEsc = constrained_pwm_lowerlimit;
  } else if (cappedEsc > constrained_pwm_upperlimit) {
	cappedEsc = constrained_pwm_upperlimit;
  }
  analogWrite(escPin, cappedEsc);
  analogWrite(servoPin, servo);
  debug_msg.throttle_output = cappedEsc;
  debug_msg.steering_output = servo;
}

void messageDrive(const race::drive_values &pwm) {
  lastDriveCommandTime = micros();
  throttle = pwm.pwm_drive;
  steering = pwm.pwm_angle;
}

void driveFlags(const race::drive_flags &flag) {
  controlOverrideFlag = flag.controlOverride;
  eStopFlag = flag.eStop;
  if (eStopFlag) {
	set(pwm_center_value, pwm_center_steering_value);
  }
}

double arduino_map(int x, double in_min, double in_max, double out_min, double out_max) {
  int val = x;
  if (val < in_min) {
	val = in_min;
  } else if (val > in_max) {
	val = in_max;
  }
  return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void updateOutput() {
  long currentTime = micros();
  if ((((currentTime - lastDriveCommandTime) > 1000000) && !controlOverrideFlag) || eStopFlag) {
	set(pwm_center_value, pwm_center_steering_value);
  } else if (controlOverrideFlag) {
	int steerOut = (int) arduino_map(steeringPWMInput, 1000, 2000, 6554, 13108);
	int throttleOut = (int) arduino_map(throttlePWMInput, 1000, 2000, 6554, 13108);
	set(throttleOut, steerOut);
  } else {
    int input = 0;
	if (throttle > 9830) {
	  input = 1500 + math.fabs((throttlePWMInput - 1500))
	} else {

	}
	int throttleOut = (int) arduino_map(throttlePWMInput, 1000, 2000, 6554, 13108);
	set(throttleOut, steering);
  }
  debug_msg.throttle_input = throttlePWMInput;
  debug_msg.steering_input = steeringPWMInput;
  debug_msg.controlOverride = controlOverrideFlag;
  debug_msg.eStop = eStopFlag;

  if ((currentTime - lastDebugMsg) > 200000) {
	chatter.publish(&debug_msg);
	lastDebugMsg = currentTime;
  }
  if ((currentTime - lastDriveCommandTime) > 10000000) {
	controlOverrideFlag = true;
  }
}

ros::Subscriber<race::drive_values> sub_drive("drive_pwm", &messageDrive); // Subscribe to drive_pwm topic sent by Jetson
ros::Subscriber<race::drive_flags> sub_flag("driveFlags", &driveFlags);  // Subscribe to estop topic sent by Jetson
void setup() {
  analogWriteFrequency(escPin, 100); //  freq at which PWM signals is generated at pin 5.
  analogWriteFrequency(servoPin, 100);
  analogWriteResolution(16);                     // Resolution for the PWM signal
  analogWrite(escPin, pwm_center_value); // Setup zero velocity and steering.
  analogWrite(servoPin, pwm_center_value - steering_trim);
  nh.initNode();           // intialize ROS node
  nh.subscribe(sub_drive); // start the subscribers.
  nh.subscribe(sub_flag);
  nh.advertise(chatter); // start the publisher..can be used for debugging.
  lastDriveCommandTime = micros();
  lastDebugMsg = micros();
  attachInterrupt(escControl, risingThrottle, RISING);
  attachInterrupt(servoControl, risingSteering, RISING);
}

void loop() {
  nh.spinOnce();
  updateOutput();
}

void risingThrottle() {
  attachInterrupt(escControl, fallingThrottle, FALLING);
  throttlePWMPrevTime = micros();
}
void risingSteering() {
  attachInterrupt(servoControl, fallingSteering, FALLING);
  steeringPWMPrevTime = micros();
}
void fallingThrottle() {
  attachInterrupt(escControl, risingThrottle, RISING);
  throttlePWMInput = micros() - throttlePWMPrevTime;
  if (throttlePWMInput < 1490 || throttlePWMInput > 1510) {
	//controlOverrideFlag = true;
  }
}
void fallingSteering() {
  attachInterrupt(servoControl, risingSteering, RISING);
  steeringPWMInput = micros() - steeringPWMPrevTime;
  if (steeringPWMInput < 1424 || steeringPWMInput > 1444) {
	//controlOverrideFlag = true;
  }
}