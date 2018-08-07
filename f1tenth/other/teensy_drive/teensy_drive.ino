#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <race/chatter_values.h>
#include <race/drive_values.h>
ros::NodeHandle nh;

boolean eStopFlag = false; // These values were cacluated for the specific Teensy microcontroller using an oscilloscope.
boolean controlOverrideFlag = false;
int pwm_center_value = 9830; //  15% duty cycle - corresponds to zero velocity, zero steering
int pwm_lowerlimit = 8738;   //  10% duty cycle - corresponds to max reverse velocity, extreme left steering
int pwm_upperlimit = 10922;  //  20% duty cycle - corresponds to max forward velocity, extreme right steering
volatile int steeringPWMInput = 0;
volatile int steeringPWMPrevTime = 0;
volatile int throttlePWMInput = 0;
volatile int throttlePWMPrevTime = 0;
race::chatter_values debug_msg; // creater a ROS Publisher called chatter of type debug_msg
ros::Publisher chatter("chatter", &debug_msg);
volatile long lastDriveCommandTime = 0;
volatile long lastDebugMsg = 0;
volatile int throttle = 0;
volatile int steering = 0;
int throttleOutPin = 5;
int steeringOutPin = 6;
int throttleInPin = 2;
int steeringInPin = 3;

// Pin 5 is connected to the ESC..dive motor
// Pin 6 is connected to the steering servo.
// Pin 2 is throttle input from the Reciever
// Pin 3 is the steering input from the Reciever

void messageDrive(const race::drive_values &pwm) {
  lastDriveCommandTime = micros();
  if (pwm.pwm_drive < pwm_lowerlimit) {
    throttle = pwm_center_value;
  } else if (pwm.pwm_drive > pwm_upperlimit) {
    throttle = pwm_center_value;
  } else {
    throttle = pwm.pwm_drive;
  }

  if (pwm.pwm_angle < pwm_lowerlimit) {
    steering = pwm_lowerlimit;
  } else if (pwm.pwm_angle > pwm_upperlimit) {
    steering = pwm_upperlimit;
  } else {
    steering = pwm.pwm_angle;
  }
}

void messageEmergencyStop(const std_msgs::Bool &flag) {
  eStopFlag = flag.data;
  if (eStopFlag == true) {
    analogWrite(5, pwm_center_value);
    analogWrite(6, pwm_center_value);
  }
}
void updateOutput() {
  long currentTime = micros();
  if ((((currentTime - lastDriveCommandTime) > 3000000) && !controlOverrideFlag) || eStopFlag) {
    analogWrite(throttleOutPin, pwm_center_value);
    analogWrite(steeringOutPin, pwm_center_value);
    debug_msg.throttle_output = pwm_center_value;
    debug_msg.steering_output = pwm_center_value;
  } else if (controlOverrideFlag) { ;
    analogWrite(throttleOutPin, throttlePWMInput);
    analogWrite(steeringOutPin, seeringPWMInput);
    debug_msg.throttle_output = throttlePWMInput;
    debug_msg.steering_output = steeringPWMInput;
  } else {
    analogWrite(throttleOutPin, throttle);
    analogWrite(steeringOutPin, steering);
    debug_msg.throttle_output = throttle;
    debug_msg.steering_output = steering;
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

void setup() {
  // Need to produce PWM signals so we need to setup the PWM registers. This setup happens next.
  analogWriteFrequency(throttleOutPin, 100); //  freq at which PWM signals is generated at pin 5.
  analogWriteFrequency(steeringOutPin, 100);
  analogWriteResolution(16);                     // Resolution for the PWM signal
  analogWrite(throttleOutPin, pwm_center_value); // Setup zero velocity and steering.
  analogWrite(steeringOutPin, pwm_center_value);
  attachInterrupt(throttleInPin, risingThrottle, RISING);
  attachInterrupt(steeringInPin, risingSteering, RISING);
  nh.initNode();           // intialize ROS node
  nh.subscribe(sub_drive); // start the subscribers.
  nh.subscribe(sub_stop);
  nh.advertise(chatter); // start the publisher..can be used for debugging.
  lastDriveCommandTime = micros();
  lastDebugMsg = micros();
}

void loop() {
  nh.spinOnce();
  updateOutput();
}

void risingThrottle() {
  attachInterrupt(throttleInPin, fallingThrottle, FALLING);
  throttlePWMPrevTime = micros();
}
void risingSteering() {
  attachInterrupt(steeringInPin, fallingSteering, FALLING);
  steeringPWMPrevTime = micros();
}
void fallingThrottle() {
  attachInterrupt(throttleInPin, risingThrottle, RISING);
  throttlePWMInput = micros() - throttlePWMPrevTime;
  if (throttlePWMInput < 1475 || throttlePWMInput > 1525) {
    controlOverrideFlag = true;
  }
}
void fallingSteering() {
  attachInterrupt(steeringInPin, risingSteering, RISING);
  steeringPWMInput = micros() - steeringPWMPrevTime;
  if (steeringPWMInput < 1419 || steeringPWMInput > 1469) {
    controlOverrideFlag = true;
  }
}
