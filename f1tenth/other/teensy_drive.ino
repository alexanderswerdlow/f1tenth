#include <ros.h>                // header files sourced from  Step 3
#include <std_msgs/Bool.h>      
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <race/drive_values.h>
ros::NodeHandle  nh;


boolean flagStop = false;     // These values were cacluated for the specific Teensy microcontroller using an oscilloscope. 
int pwm_center_value = 9830;  //  15% duty cycle - corresponds to zero velocity, zero steering
int pwm_lowerlimit = 6554;    //  10% duty cycle - corresponds to max reverse velocity, extreme left steering
int pwm_upperlimit = 13108;   //  20% duty cycle - corresponds to max forward velocity, extreme right steering
volatile int pwm_value = 0;
volatile int prev_time = 0;
volatile int pwm_value1 = 0;
volatile int prev_time1 = 0;
std_msgs::Int32 str_msg;          // creater a ROS Publisher called chatter of type str_msg
ros::Publisher chatter("chatter", &str_msg);

void messageDrive( const race::drive_values& pwm ) 
{
    str_msg.data = pwm_value;
    chatter.publish( &str_msg );

  if(flagStop == false)
  {


    if(pwm.pwm_drive < pwm_lowerlimit)  // Pin 5 is connected to the ESC..dive motor
    {
      analogWrite(5,pwm_lowerlimit);    //  Safety lower limit        
    }
    else if(pwm.pwm_drive > pwm_upperlimit)
    {
      analogWrite(5,pwm_upperlimit);    //  Safety upper limit
    }
    else
    {
      analogWrite(5,pwm.pwm_drive);     //  Incoming data                    
    }

    
    if(pwm.pwm_angle < pwm_lowerlimit) // Pin 6 is connected to the steering servo.
    {
      analogWrite(6,pwm_lowerlimit);    //  Safety lower limit        
    }
    else if(pwm.pwm_angle > pwm_upperlimit)
    {
      analogWrite(6,pwm_upperlimit);    //  Safety upper limit
    }
    else
    {
      analogWrite(6,pwm.pwm_angle);     //  Incoming data                    
    }

  }
  else
  {
    analogWrite(5,pwm_center_value);
    analogWrite(6,pwm_center_value);    
  }
}

void messageEmergencyStop( const std_msgs::Bool& flag )
{
  flagStop = flag.data;
  if(flagStop == true)
  {
    analogWrite(5,pwm_center_value);
    analogWrite(6,pwm_center_value);    
  }
}


ros::Subscriber<race::drive_values> sub_drive("drive_pwm", &messageDrive );   // Subscribe to drive_pwm topic sent by Jetson
ros::Subscriber<std_msgs::Bool> sub_stop("eStop", &messageEmergencyStop );  // Subscribe to estop topic sent by Jetson

void setup() {
  // Need to produce PWM signals so we need to setup the PWM registers. This setup happens next.
  analogWriteFrequency(5, 100); //  freq at which PWM signals is generated at pin 5.
  analogWriteFrequency(6, 100); 
  analogWriteResolution(16); // Resolution for the PWM signal
  analogWrite(5,pwm_center_value); // Setup zero velocity and steering.
  analogWrite(6,pwm_center_value);
  attachInterrupt(2,rising,RISING);
  attachInterrupt(3,rising1,RISING);
  nh.initNode();  // intialize ROS node
  nh.subscribe(sub_drive); // start the subscribers.
  nh.subscribe(sub_stop);
  nh.advertise(chatter);  // start the publisher..can be used for debugging.

}

void loop() {
  nh.spinOnce();
}

void rising(){
  attachInterrupt(2,falling,FALLING);
  prev_time = micros();
}
void rising1(){
  attachInterrupt(3,falling1,FALLING);
  prev_time1 = micros();
}
void falling1(){
  attachInterrupt(3,rising1,RISING);
  pwm_value1 = micros() - prev_time1;
  if(pwm_value1 < 1475 || pwm_value1 > 1525){
    flagStop = true;
    analogWrite(5,pwm_center_value);
    analogWrite(6,pwm_center_value);
  }
}

void falling(){
  attachInterrupt(2,rising,RISING);
  pwm_value = micros() - prev_time;
  if(pwm_value < 1475 || pwm_value > 1525){
    flagStop = true;
    analogWrite(5,pwm_center_value);
    analogWrite(6,pwm_center_value);
  }
}


