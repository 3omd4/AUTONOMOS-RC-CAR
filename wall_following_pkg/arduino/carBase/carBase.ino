/* 
  RC Car controller (using RosSerial).
  Arduino is considered as ROS Node { LOW level control }.
  This Node is a interface between Jetson , Motors , sensors . 

  > Topic           - Msg                           - purpose 
 
  > Subscribers :  
  > /Cmd_Vel        - std_msgs/Float32              - Controlling DC Motor control For speed 
  > /Cmd_Steering   - std_msgs/Float32              - Controlling The angle of servo For steering
  > /PID_Gains      - std_msgs/Float32MultiArray    - Taking the new Gains from jetson (kp & kd)

  > Publishers : 
  > /RPM            - std_msgs/Float32      - For encoder velocity

  > Display position and orientation on (16x2 I2C LCD)
  > shows : PWM , Vel (m/s) , RPM
*/

//-------------------------------------- libraries -----------------------------------

#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include "Wire.h"
#include <math.h>

//-----------------------------------------------------------------------------------------

// Pins
const int IR_SENSOR_PIN = 2;  // Interrupt for encoder
const int MOTOR_ENA_PIN = 3;  // Digital pin > PWM  
const int MOTOR_IN1_PIN = 5;  // Digital pin
const int MOTOR_IN2_PIN = 6;  // Digital pin
const int SERVO_PIN = 9;      // Digital pin > PWM  (send angle direct)

// Servo limits
const int SERVO_MIN = 78;     
const int SERVO_CENTER = 90;
const int SERVO_MAX = 102;

// Encoder
const int SLOTS_PER_REV = 20;

// Publishing intervals
const unsigned long RPM_INTERVAL_MS = 1000;

// Objects
LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo steeringServo;

// ROS Topics and Publisher
ros::NodeHandle nh;
std_msgs::Float32 vel_msg;
ros::Publisher velocity_pub("Velocity", &vel_msg);

volatile unsigned long pulseCount = 0;
#define M_PI 3.1416
unsigned long lastRpmTime = 0;

float currentRpm = 0.0;
float vel = 0.0;
float target_speed = 0.0 ;

float wheel_Dia = 0.064; // m
float wheelCircumference = M_PI*wheel_Dia;

float error = 0.0 ; 
float last_error = 0.0 ; 
unsigned long delta_time_ms = 0 ; 
float delta_error = 0.0 ; 
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
void cmdVelCallback(const std_msgs::Float32& required_speed) {   // required speed > m/s
  target_speed = required_speed.data ;
}


void PIDGainsCallback(const std_msgs::Float32MultiArray& msg_Gains) {
  if (msg_Gains.data_length == 2)
  {
    kp = msg_Gains.data[0];
    kd = msg_Gains.data[1];
  }
}


void cmdSteeringCallback(const std_msgs::Float32& steering_cmd) {
  int servo_angle = (int)(steering_cmd.data);
  setSteering(servo_angle);
}

ros::Subscriber<std_msgs::Float32MultiArray> cmd_pid_sub("PID_Gains", &PIDGainsCallback);
ros::Subscriber<std_msgs::Float32> cmd_vel_sub("cmd_vel", &cmdVelCallback);
ros::Subscriber<std_msgs::Float32> cmd_steer_sub("cmd_steering", &cmdSteeringCallback);


// =================================================================================================
// ===============================                SETUP                       ======================
// =================================================================================================

void setup() {
  Serial.begin(57600); // Works reliably with Uno Rosserial
  Wire.begin();
       

  // LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print(F("Arduino Ready .."));

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
  nh.subscribe(cmd_vel_sub);
  nh.subscribe(cmd_steer_sub);
  nh.subscribe(cmd_pid_sub);

  lcd.setCursor(0, 1);
  lcd.print(F("ROS Ready!"));
  delay(500);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("PWM: "));
  lcd.setCursor(8, 0);
  lcd.print(F("RPM: "));
  lcd.setCursor(0, 1);
  lcd.print(F("Vel: "));
  
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

  //--------- PID control -------------
  dt_pid = (now-lastPidtime )/1000.0f ;
  error = target_speed - vel ;
  delta_error = error - last_error ;
  
  if (dt_pid > 0.0f)
  { derivative =  (kd * delta_error)/dt_pid  ; }
  else 
  { derivative = 0 ; }

  motor_PWM =  (int)round( (kp*error)  + derivative ) ; 
  motor_PWM = constrain(motor_PWM , -255 , 255 ) ; 
  setMotorSpeed(motor_PWM);
  
  last_error = error ; 
  lastPidtime = now ;

  // Update LCD
  lcd.setCursor(4, 0);
  lcd.print(currentRpm, 1);
  lcd.print("  ");
  lcd.setCursor(13, 0);
  lcd.print(vel, 2);
  lcd.print("  ");
  lcd.setCursor(5, 1);
  lcd.print(motor_PWM );
  lcd.print("  ");


  delay(50); // ~20Hz loop
}

