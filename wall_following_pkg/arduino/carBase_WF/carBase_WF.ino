/* 
  RC Car controller (using RosSerial).
  Arduino is considered as ROS Node { LOW level control }.
  This Node is a interface between Jetson , Motors , sensors . 

  > Topic           - Msg                           - purpose 

  > Subscribers :  
  > /Cmd_Steer      	- std_msgs/Float32              - Controlling The angle of servo For steering
  > /cmd_pwm      	- std_msgs/Float32              - Controlling The PWM

  > Publishers : 
  > /vel            	- std_msgs/Float32      - For encoder velocity
*/

//-------------------------------------- libraries -----------------------------------

#include "I2Cdev.h"
#include "MPU6050.h"
#include <Servo.h>
#define ROSLIB_SERIAL_SIZE 32
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

#include "Wire.h"
#include <math.h>

//-----------------------------------------------------------------------------------------

// Pins
const uint8_t IR_SENSOR_PIN = 2;  // Interrupt for encoder
const uint8_t MOTOR_ENA_PIN = 3;  // Digital pin > PWM  
const uint8_t MOTOR_IN1_PIN = 5;  // Digital pin
const uint8_t MOTOR_IN2_PIN = 6;  // Digital pin
const uint8_t SERVO_PIN = 9;      // Digital pin > PWM  (send angle direct)

// Servo limits
const uint8_t SERVO_MIN = 78;     
const uint8_t SERVO_CENTER = 95;
const uint8_t SERVO_MAX = 112;

// Encoder
const uint8_t SLOTS_PER_REV = 20;

// Publishing intervals
const unsigned long RPM_INTERVAL_MS = 1000;

// ROS Topics and Publisher
ros::NodeHandle nh;
std_msgs::Float32 vel_msg;
ros::Publisher velocity_pub("/Vel", &vel_msg);

// Objects
Servo steeringServo;


// -------------------------- Variables -----------------------------
volatile unsigned long pulseCount = 0;
unsigned long lastRpmTime = 0;

float currentRpm = 0.0;
float vel = 0.0;

const float wheel_Dia = 0.064; // m
const float wheelCircumference = M_PI*wheel_Dia;

unsigned long delta_time_ms = 0 ; 
int   motor_PWM = 0 ;

// ISR for encoder
void countPulse() {
  pulseCount++;
}


//-------------------------------------- LOW level Functions -----------------------------------

// PID  
float dt_pid = 0.0 ; 
unsigned long lastPidtime = 0 ; 
float derivative = 0.0  ;
float kp = 0.0 ;
float kd = 0.0 ;
 
// Motor control
void setMotorSpeed(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(MOTOR_IN1_PIN, HIGH);
    digitalWrite(MOTOR_IN2_PIN, LOW);
    analogWrite(MOTOR_ENA_PIN, speed);
  } else if (speed < 0) {
    digitalWrite(MOTOR_IN1_PIN, LOW);
    digitalWrite(MOTOR_IN2_PIN, HIGH);
    analogWrite(MOTOR_ENA_PIN, -speed);
  } else {
    digitalWrite(MOTOR_IN1_PIN, LOW);
    digitalWrite(MOTOR_IN2_PIN, LOW);
    analogWrite(MOTOR_ENA_PIN, 0);
  }
}

// Servo control
void setSteering(int angle) {
  steeringServo.write(constrain(angle, SERVO_MIN, SERVO_MAX));
}

//---------------------------------- ROS Subscribers callbacks --------------------------------------

void cmdSteeringCallback(const std_msgs::Float32& steering_cmd) {
  int servo_angle = (int)(steering_cmd.data);
  setSteering(servo_angle);
}

void cmdpwmCallback(const std_msgs::Float32& vel_cmd) {
  int pwm = (int)(vel_cmd.data);
  setMotorSpeed(pwm);
}


ros::Subscriber<std_msgs::Float32> cmd_steer_sub("/cmd_steer", &cmdSteeringCallback);
ros::Subscriber<std_msgs::Float32> cmd_pwm_sub("/cmd_pwm", &cmdpwmCallback);


// =================================================================================================
// ===============================                SETUP                       ======================
// =================================================================================================

void setup() {
  Serial.begin(57600); // Works reliably with Uno Rosserial
  Wire.begin();
       
  // Servo
  steeringServo.attach(SERVO_PIN);
  steeringServo.write(SERVO_CENTER);   // set steering forward 

  // Motor
  pinMode(MOTOR_ENA_PIN, OUTPUT);
  pinMode(MOTOR_IN1_PIN, OUTPUT);
  pinMode(MOTOR_IN2_PIN, OUTPUT);
  setMotorSpeed(0);

  // Encoder
  pinMode(IR_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(IR_SENSOR_PIN), countPulse, RISING);

  // ROS
  nh.initNode();
  nh.advertise(velocity_pub);
  nh.subscribe(cmd_steer_sub);
  nh.subscribe(cmd_pwm_sub);

  delay(500);
  
  lastPidtime = millis();
  lastRpmTime = millis();
}

// =================================================================================================
// ===============================                LOOP                       ======================
// =================================================================================================

void loop() {
  nh.spinOnce();
  unsigned long now = millis();
  delta_time_ms = (now - lastRpmTime) ;

  //--------- RPM calculation -------------
  if ( delta_time_ms >= RPM_INTERVAL_MS) {
    noInterrupts();
    unsigned long pulses = pulseCount;
    pulseCount = 0;
    interrupts();

    currentRpm = (pulses / (float)SLOTS_PER_REV) * 60.0; // RPM speed
    vel = ( currentRpm * wheelCircumference )/ 60 ;      // m/s speed
    vel_msg.data = vel ;
    velocity_pub.publish(&vel_msg);

    lastRpmTime = now;
  }

  delay(50); // ~20Hz loop
}
