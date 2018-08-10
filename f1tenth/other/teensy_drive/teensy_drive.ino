#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <race/chatter_values.h>
#include <race/drive_values.h>
ros::NodeHandle nh;
race::chatter_values debug_msg; // creater a ROS Publisher called chatter of type debug_msg
ros::Publisher chatter("chatter", &debug_msg);

boolean eStopFlag = false;
boolean controlOverrideFlag = false;
int pwm_center_value = 9830;
int steering_trim = 433;
int pwm_center_steering_value = pwm_center_value - steering_trim;
int constrained_pwm_lowerlimit = 7500;
int constrained_pwm_upperlimit = 1200;
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
  analogWrite(escPin, esc);
  analogWrite(servoPin, servo);
  debug_msg.throttle_output = esc;
  debug_msg.steering_output = servo;
}

void messageDrive(const race::drive_values &pwm) {
  lastDriveCommandTime = micros();
  throttle = pwm.pwm_drive;
  steering = pwm.pwm_angle;
}

void messageEmergencyStop(const std_msgs::Bool &flag) {
  eStopFlag = flag.data;
  if (eStopFlag) {
	set(pwm_center_value, pwm_center_steering_value);
  }
}

void overrideControl(const std_msgs::Bool &flag) {
  controlOverrideFlag = flag.data;
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
  if ((((currentTime - lastDriveCommandTime) > 2000000) && !controlOverrideFlag) || eStopFlag) {
	set(pwm_center_value, pwm_center_steering_value);
  } else if (controlOverrideFlag) {
	int steerOut = (int) arduino_map(steeringPWMInput, 1000, 2000, 6554, 13108);
	int throttleOut = (int) arduino_map(throttlePWMInput, 1000, 2000, 6554, 13108);
	set(throttleOut, steerOut);
  } else {
	set(throttle, steering);
  }
  debug_msg.throttle_input = throttlePWMInput;
  debug_msg.steering_input = steeringPWMInput;
  debug_msg.controlOverride = controlOverrideFlag;
  debug_msg.eStop = eStopFlag;

  if ((currentTime - lastDebugMsg) > 50000) {
	chatter.publish(&debug_msg);
	lastDebugMsg = currentTime;
  }
}

ros::Subscriber<race::drive_values> sub_drive("drive_pwm", &messageDrive); // Subscribe to drive_pwm topic sent by Jetson
ros::Subscriber<std_msgs::Bool> sub_stop("eStop", &messageEmergencyStop);  // Subscribe to estop topic sent by Jetson
ros::Subscriber<std_msgs::Bool> sub_control("controlOverride", &overrideControl);  // Subscribe to estop topic sent by Jetson

void setup() {
  analogWriteFrequency(escPin, 100); //  freq at which PWM signals is generated at pin 5.
  analogWriteFrequency(servoPin, 100);
  analogWriteResolution(16);                     // Resolution for the PWM signal
  analogWrite(escPin, pwm_center_value); // Setup zero velocity and steering.
  analogWrite(servoPin, pwm_center_value - steering_trim);
  attachInterrupt(escControl, risingThrottle, RISING);
  attachInterrupt(servoControl, risingSteering, RISING);
  nh.initNode();           // intialize ROS node
  nh.subscribe(sub_drive); // start the subscribers.
  nh.subscribe(sub_stop);
  nh.subscribe(sub_control);
  nh.advertise(chatter); // start the publisher..can be used for debugging.
  lastDriveCommandTime = micros();
  lastDebugMsg = micros();
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
	controlOverrideFlag = true;
  }
}
void fallingSteering() {
  attachInterrupt(servoControl, risingSteering, RISING);
  steeringPWMInput = micros() - steeringPWMPrevTime;
  if (steeringPWMInput < 1424 || steeringPWMInput > 1444) {
	controlOverrideFlag = true;
  }
}