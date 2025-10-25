/* 
  RC Car controller (using RosSerial).
  Arduino is considered as ROS Node { LOW level control }.
  This Node is a interface between Jetson , Motors , sensors . 

  > Topic           - Msg                   - purpose 
 
  > Subscribers :  
  > /Cmd_Vel        - std_msgs/Float32   - Controlling DC Motor control For speed 
  > /Cmd_Steering   - std_msgs/Float32   - Controlling The angle of servo For steering

  > Publishers : 
  > /Encoder_Ticks  - std_msgs/Float32      - For encoder ticks 
  > /RPM            - std_msgs/Float32      - For encoder velocity
  > /IMU            - sensor_msgs/Imu       - For orientation & gyro & accelometer

  > Display position and orientation on (16x2 I2C LCD)
  > shows : YAW (deg) , Vel (m/s) , RPM
*/


//-------------------------------------- libraries -----------------------------------

#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file >> already exist with me >_<
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


//------------------------------------------- MPU 6050 DMP -----------------------------------------
// "OUTPUT_READABLE_YAWPITCHROLL"  yaw/ pitch/roll angles (in degrees) 
// calculated from the quaternions coming from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock 

// "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format 


// "OUTPUT_READABLE_REALACCEL" if you want to see acceleration components with gravity removed. 
// This acceleration reference frame is not compensated for orientation, so +X is always +X according to the sensor,
// just without the effects of gravity. 


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

const float GYRO_LSB_PER_DEG = 65.5f ;      // LSB per °/s for ±500 °/s (from datasheet)
const float ACCEL_LSB_PER_G   = 4096.0f;    // LSB per g for ±8 g (from datasheet)
const float G_TO_MS2         = 9.80665f;    // g -> m/s^2
const float DEG_TO_RAD       = 3.1416f / 180.0f;

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
MPU6050 mpu;


// ROS Topics and Publisher
ros::NodeHandle nh;
std_msgs::Float32 ticks_msg;
ros::Publisher ticks_pub("Encoder_Ticks", &ticks_msg);
std_msgs::Float32 vel_msg;
ros::Publisher velocity_pub("Velocity", &vel_msg);
std_msgs::Imu imu_msg;
ros::Publisher imu_pub("Imu", &imu_msg);

volatile unsigned long pulseCount = 0;
#define M_PI 3.1416
unsigned long lastRpmTime = 0;
float currentRpm = 0;
float vel = 0;
float wheel_Dia = 0.065; // m
float wheelCircumference = M_PI*wheel_Dia;

// ISR for encoder
void countPulse() {
  pulseCount++;
}


unsigned long last_publish = 0;
const unsigned long publish_interval = 10; // 10 ms = 100 Hz


//-------------------------------------- LOW level Functions -----------------------------------
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
// ===============================                SETUP                       ======================
// =================================================================================================

void setup() {
  Serial.begin(57600); // Works reliably with Uno Rosserial

  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        //Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif


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

  // MPU
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("initializing I2C"));
  mpu.initialize();

  // verifiy connection 
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Testing MPU"));

  if (mpu.testConnection())
  {
      lcd.setCursor(0, 1);
      lcd.print(F("Connected"));   
      delay(1000); 

      }
  else 
  {      
      lcd.setCursor(0, 1);
      lcd.print(F("Failed:("));  
      delay(1000); 
      }


  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);  // ± 500 deg/sec
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);  // ± 8 g  ( since g = 9.81 )
  
  devStatus = mpu.dmpInitialize();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Init DMP ..."));
  delay(500) ; 


  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(163);
  mpu.setYGyroOffset(28);
  mpu.setZGyroOffset(50);
  mpu.setZAccelOffset(1252); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
      // Calibration Time: generate offsets and calibrate our MPU6050
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("Calibrating MPU"));
      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
      mpu.PrintActiveOffsets();
      // turn on the DMP, now that it's ready
      
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("Enabling DMP"));
      mpu.setDMPEnabled(true);

      dmpReady = true;
      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("DMP Ready!");
  } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("DMP Failed!");
      lcd.setCursor(0, 1);
      lcd.print(devStatus);
      delay(1000);
  }

  // ROS
  nh.initNode();
  nh.advertise(velocity_pub);
  nh.advertise(ticks_pub);
  nh.advertise(imu_pub);
  nh.subscribe(cmd_sub);
  nh.subscribe(cmd_steer_sub);

  lcd.setCursor(0, 1);
  lcd.print(F("ROS Ready!"));
  delay(500);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("RPM: "));
  lcd.setCursor(8, 0);
  lcd.print(F("Vel: "));
  lcd.setCursor(0, 1);
  lcd.print(F("Yaw: "));

}

// =================================================================================================
// ===============================                LOOP                       ======================
// =================================================================================================

void loop() {
  nh.spinOnce();
  unsigned long now = millis();

  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { 
      // Get the Latest packet 
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGyro(&gy, fifoBuffer);    
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

      // ----------------------------------- conversions and constants  -------------------------------------------- 
      if (now - last_publish >= publish_interval) {
          imu_pub.publish(&imu_msg);
          last_publish = now;
      
      // Convert gyro raw -> rad/s
      float gx_dps = (float)gy.x / GYRO_LSB_PER_DEG; // degrees/sec
      float gy_dps = (float)gy.y / GYRO_LSB_PER_DEG;
      float gz_dps = (float)gy.z / GYRO_LSB_PER_DEG;

      float gx_rad = gx_dps * DEG_TO_RAD; // rad/s
      float gy_rad = gy_dps * DEG_TO_RAD;
      float gz_rad = gz_dps * DEG_TO_RAD;

      // Convert linear accel raw (gravity-removed) -> m/s^2
      float ax_g = (float)aaReal.x / ACCEL_LSB_PER_G; // in g
      float ay_g = (float)aaReal.y / ACCEL_LSB_PER_G;
      float az_g = (float)aaReal.z / ACCEL_LSB_PER_G;

      float ax_ms2 = ax_g * G_TO_MS2;
      float ay_ms2 = ay_g * G_TO_MS2;
      float az_ms2 = az_g * G_TO_MS2;

      imu_msg.header.stamp = nh.now();
      imu_msg.header.frame_id = "imu_link"; 

      // Orientation (quaternion) from DMP (q.w, q.x, q.y, q.z)
      imu_msg.orientation.w = q.w;
      imu_msg.orientation.x = q.x;
      imu_msg.orientation.y = q.y;
      imu_msg.orientation.z = q.z;

      // Angular velocity (rad/s)
      imu_msg.angular_velocity.x = gx_rad;
      imu_msg.angular_velocity.y = gy_rad;
      imu_msg.angular_velocity.z = gz_rad;

      // Linear acceleration (m/s^2) - gravity removed (aaReal expected)
      imu_msg.linear_acceleration.x = ax_ms2;
      imu_msg.linear_acceleration.y = ay_ms2;
      imu_msg.linear_acceleration.z = az_ms2;

      // Covariances tuned for a 1:10 RC car (initial guesses — tune later):
      // - Orientation covariance: moderate (DMP gives good quaternions but not perfect)
      // - Angular velocity covariance: small (gyro is relatively precise after scale choice)
      // - Linear acceleration covariance: larger (accelerometer noisy and affected by vibrations)
      for (int i = 0; i < 9; i++) {
        imu_msg.orientation_covariance[i]        = 0.01;   // rad^2 (example)
        imu_msg.angular_velocity_covariance[i]   = 0.001;  // (rad/s)^2
        imu_msg.linear_acceleration_covariance[i]= 0.25;   // (m/s^2)^2
      }

      // If you prefer to mark any covariance as "unknown" use -1 in the [0] element:
      // imu_msg.orientation_covariance[0] = -1.0;

      // Publish IMU
      imu_pub.publish(&imu_msg);
  }


  // RPM calculation
  if (now - lastRpmTime >= RPM_INTERVAL_MS) {
    noInterrupts();
    unsigned long pulses = pulseCount;
    pulseCount = 0;
    interrupts();

    ticks_msg.data = pulses ;
    ticks_pub.publish(&ticks_msg);

    currentRpm = (pulses / (float)SLOTS_PER_REV) * 60.0; // RPM speed
    vel = ( currentRpm * wheelCircumference )/ 60 ;      // m/s speed
    vel_msg.data = vel ;
    velocity_pub.publish(&vel_msg);

    // Update LCD
    lcd.setCursor(4, 0);
    lcd.print(currentRpm, 1);
    lcd.print("  ");
    lcd.setCursor(13, 0);
    lcd.print(vel, 2);
    lcd.print("  ");
    lcd.setCursor(5, 1);
    lcd.print(ypr[0] * 180.0 / M_PI, 1);
    lcd.print("  ");

    lastRpmTime = now;
  }

  delay(50); // ~20Hz loop
}

