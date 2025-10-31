/* RC Car controller (using RosSerial).
  Arduino is considered as ROS Node { LOW level control }.
  This Node is a interface between Jetson , Motors , sensors . 

  > Topic         - Msg                 - purpose 
 
  > Subscribers :  
  > /Cmd_Vel        - std_msgs/Float32    - Controlling DC Motor control For speed 
  > /Cmd_Steering   - std_msgs/Float32    - Controlling The angle of servo For steering

  > Publishers : 
  > /Encoder_Ticks  - std_msgs/Float32      - For encoder ticks 
  > /RPM            - std_msgs/Float32      - For encoder velocity
  > /IMU            - sensor_msgs/Imu       - For orientation & gyro & accelometer

  > Display position and orientation on (16x2 I2C LCD)
  > shows : YAW (deg) , Vel (m/s) , RPM
*/


//-------------------------------------- libraries -----------------------------------

// #include <LiquidCrystal_I2C.h> // <-- REMOVED
#include <Servo.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

//------------------------------------------- MPU 6050 DMP -----------------------------------------
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[42];

Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorFloat gravity;
VectorInt16 gy;
float ypr[3];

#define GYRO_LSB_PER_DEG 65.5f
#define ACCEL_LSB_PER_G 4096.0f
#define G_TO_MS2 9.80665f
//-----------------------------------------------------------------------------------------

const int IR_SENSOR_PIN = 2;
const int MOTOR_ENA_PIN = 3;
const int MOTOR_IN1_PIN = 5;
const int MOTOR_IN2_PIN = 6;
const int SERVO_PIN = 9;

const int SERVO_MIN = 78;
const int SERVO_CENTER = 90;
const int SERVO_MAX = 102;

const int SLOTS_PER_REV = 20;

const unsigned long RPM_INTERVAL_MS = 1000;

Servo steeringServo;
MPU6050 mpu;

ros::NodeHandle nh;
std_msgs::Float32 ticks_msg;
ros::Publisher ticks_pub("Encoder_Ticks", &ticks_msg);
std_msgs::Float32 vel_msg;
std_msgs::Float32MultiArray imu_msg;
ros::Publisher velocity_pub("Velocity", &vel_msg);
ros::Publisher imu_pub("Imu", &imu_msg);

float imu_data[10];  // quaternion(4) + gyro(3) + accel(3)

volatile unsigned long pulseCount = 0;
unsigned long lastRpmTime = 0;
float currentRpm = 0;
float vel = 0;
float wheel_Dia = 0.065;
float wheelCircumference = 3.1416 * wheel_Dia;

void countPulse() {
  pulseCount++;
}

unsigned long last_publish = 0;
const unsigned long publish_interval = 50;

//-------------------------------------- LOW level Functions -----------------------------------
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

void setSteering(int angle) {
  steeringServo.write(constrain(angle, SERVO_MIN, SERVO_MAX));
}

//---------------------------------- ROS Subscribers callbacks --------------------------------------
void cmdVelCallback(const std_msgs::Float32& cmd) {
  int motor_speed = (int)(cmd.data * 255.0);
  setMotorSpeed(motor_speed);
}

void cmdSteeringCallback(const std_msgs::Float32& cmd) {
  int servo_angle = (int)(cmd.data);
  setSteering(servo_angle);
}

ros::Subscriber<std_msgs::Float32> cmd_sub("cmd_vel", &cmdVelCallback);
ros::Subscriber<std_msgs::Float32> cmd_steer_sub("cmd_steering", &cmdSteeringCallback);

// =================================================================================================
void setup() {
  Serial.begin(57600);

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  steeringServo.attach(SERVO_PIN);
  steeringServo.write(SERVO_CENTER);

  pinMode(MOTOR_ENA_PIN, OUTPUT);
  pinMode(MOTOR_IN1_PIN, OUTPUT);
  pinMode(MOTOR_IN2_PIN, OUTPUT);
  setMotorSpeed(0);

  pinMode(IR_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(IR_SENSOR_PIN), countPulse, RISING);

  mpu.initialize();

  if (mpu.testConnection()) {
    delay(1000);
  } else {
    delay(1000);
  }

  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);

  devStatus = mpu.dmpInitialize();
  delay(500);

  mpu.setXGyroOffset(163);
  mpu.setYGyroOffset(28);
  mpu.setZGyroOffset(50);
  mpu.setZAccelOffset(1252);

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    mpu.setDMPEnabled(true);
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    delay(1000);
  }

  nh.initNode();
  nh.advertise(velocity_pub);
  nh.advertise(ticks_pub);
  nh.advertise(imu_pub);
  nh.subscribe(cmd_sub);
  nh.subscribe(cmd_steer_sub);

  imu_msg.data = imu_data;
  imu_msg.data_length = 10;

  delay(500);
}

// =================================================================================================
void loop() {
  nh.spinOnce();
  unsigned long now = millis();

  if (!dmpReady) return;

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGyro(&gy, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

    if (now - last_publish >= publish_interval) {
      last_publish = now;

      float gx_dps = (float)gy.x / GYRO_LSB_PER_DEG;
      float gy_dps = (float)gy.y / GYRO_LSB_PER_DEG;
      float gz_dps = (float)gy.z / GYRO_LSB_PER_DEG;

      float gx_rad = gx_dps * DEG_TO_RAD;
      float gy_rad = gy_dps * DEG_TO_RAD;
      float gz_rad = gz_dps * DEG_TO_RAD;

      float ax_g = (float)aaReal.x / ACCEL_LSB_PER_G;
      float ay_g = (float)aaReal.y / ACCEL_LSB_PER_G;
      float az_g = (float)aaReal.z / ACCEL_LSB_PER_G;

      float ax_ms2 = ax_g * G_TO_MS2;
      float ay_ms2 = ay_g * G_TO_MS2;
      float az_ms2 = az_g * G_TO_MS2;

      imu_data[0] = q.w;
      imu_data[1] = q.x;
      imu_data[2] = q.y;
      imu_data[3] = q.z;
      imu_data[4] = gx_rad;
      imu_data[5] = gy_rad;
      imu_data[6] = gz_rad;
      imu_data[7] = ax_ms2;
      imu_data[8] = ay_ms2;
      imu_data[9] = az_ms2;

      imu_pub.publish(&imu_msg);
    }

    if (now - lastRpmTime >= RPM_INTERVAL_MS) {
      noInterrupts();
      unsigned long pulses = pulseCount;
      pulseCount = 0;
      interrupts();

      ticks_msg.data = pulses;
      ticks_pub.publish(&ticks_msg);

      currentRpm = (pulses / (float)SLOTS_PER_REV) * 60.0;
      vel = (currentRpm * wheelCircumference) / 60;
      vel_msg.data = vel;
      velocity_pub.publish(&vel_msg);

      lastRpmTime = now;
    }

    delay(50);
  }
}
