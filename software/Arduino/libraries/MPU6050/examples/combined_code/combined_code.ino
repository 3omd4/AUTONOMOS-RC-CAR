//Receives commands over serial at 115200
/*
Motor Control - M[value]

Example: M150 → Motor forward at PWM 150

Example: M-120 → Motor reverse at PWM 120

Range: -255 to 255

Servo Control - S[angle]

Example: S90 → Set steering servo to 90° center position

Example: S45 → Turn left to 45°

Typical range: 0-180° (depends on servo)

Request Data - R

Triggers the Arduino to send back all sensor readings
*/
/*
ENC:123,AX:256,AY:-124,AZ:16384,GX:120,GY:-45,GZ:200 example of messages sent
*/

#include <Servo.h>
Servo steeringServo;
const int MOTOR_PWM = 5;
const int MOTOR_IN1 = 3;
const int MOTOR_IN2 = 4;
const int ENCODER_PIN = 2;

volatile unsigned long pulseCount = 0;

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

MPU6050 mpu;


// "OUTPUT_READABLE_YAWPITCHROLL"  yaw/ pitch/roll angles (in degrees) 
// calculated from the quaternions coming from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
#define OUTPUT_READABLE_YAWPITCHROLL

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

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
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// ================================================================
// ===               INTERRUPT encoder               ===
// ================================================================

void encoderISR() {
  pulseCount++;
}

// ================================================================
// ===                      SETUP                               ===
// ================================================================

void setup() {

    pinMode(MOTOR_PWM, OUTPUT);
    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);
    steeringServo.attach(9);

    Serial.println("Arduino Ready");

    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nStarting DMP programming and demo: "));
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, RISING);
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    delay(20000);

}



// ================================================================
// ===                          LOOP                            ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { 
        // Get the Latest packet 

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }

    // --- Receive commands ---
    if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.startsWith("M")) {  // Motor control: e.g. "M150" or "M-120"
        int val = cmd.substring(1).toInt();
        motorControl(val);
    }
    else if (cmd.startsWith("S")) {  // Servo control: e.g. "S90"
        int angle = cmd.substring(1).toInt();
        steeringServo.write(angle);
    }
    else if (cmd == "R") {  // Send readings
        sendReadings(ypr[0]);
    }    }

}


void motorControl(int pwmVal) {
    bool forward = pwmVal >= 0;
    pwmVal = abs(pwmVal);
    if (pwmVal > 255) pwmVal = 255;

    digitalWrite(MOTOR_IN1, forward);
    digitalWrite(MOTOR_IN2, !forward);
    analogWrite(MOTOR_PWM, pwmVal);
}

void sendReadings(float yaw_angle) {
    // Encoder
    unsigned long count = pulseCount;
    pulseCount = 0;

    // IMU
    Serial.print("ENC:");
    Serial.print(count);
    Serial.print("\t \t   yaw \t");
    Serial.print(yaw_angle*180/M_PI);
    Serial.print("\n");

}