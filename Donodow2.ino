
/////////////////////////imu
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 mpu;
#define OUTPUT_READABLE_QUATERNION
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

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
////////////////////////////////////


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================


///////////////////////////////////////shield
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(&Wire, 0x40);

#define SERVOMIN  500 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  2400 // this is the 'maximum' pulse length count (out of 4096)
Servo servo1;
#define servopin 0
// our servo # counter
uint8_t servonum = 0;
float alpha[6]; 
const double beta[6];
float rotMatrix[3][3];
float baseJoint[6][3];
float platformJoint[6][3];
float translation[3];
float pos[3];
float rot[3];
float rotation[3];
float  legLength[6][3];
float servoAngle[6];
#define SERVO_MIN 0 
#define SERVO_MAX 0
#define PLATFORM_BASE_RADIUS 75.5
#define THETA_ANGLE 55
#define PLATFORM_TOP_RADIUS 50
#define THETA_R_ANGLE 22
#define pi  3.14159
#define PLATFORM_HEIGHT_DEFAULT 140
#define LENGTH_SERVO_ARM 15
#define LENGTH_SERVO_LEG 145
/////////////////////////////////////

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
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
    devStatus = mpu.dmpInitialize();

      // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(32);
    mpu.setYGyroOffset(17);
    mpu.setZGyroOffset(89);
    mpu.setZAccelOffset(1009); // 1688 factory default for my test chip


    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
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
  Serial.println("8 channel Servo test!");

  pwm.begin();
  
  pwm.setPWMFreq(50);  // Analog servos run at ~60 Hz updates  

  getPlatformJoints();
  pwm.setPWM(0, 0, pulsewidth(135));
  pwm.setPWM(2, 0, pulsewidth(135));
  pwm.setPWM(4, 0, pulsewidth(135));
  pwm.setPWM(6, 0, pulsewidth(135));
  pwm.setPWM(8, 0, pulsewidth(135));
  pwm.setPWM(10, 0, pulsewidth(135));
  delay(1000);
}

int pulsewidth(int angle)
{
  int pulse_width, anglog_value;
  pulse_width = map(angle, 0, 270, SERVOMIN, SERVOMAX);
  anglog_value = int (float (pulse_width)/1000000*50*4096);
  return anglog_value;
}

void loop() {
      

    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
        #endif
        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
    setRotation(ypr);
    getRotationMatrix(rotMatrix);    
    setTranslation();
    calcLegLength();
    calServoAngle(servoAngle);
}


float degToRad(int angle){
  return angle/180*pi;  
}
void getPlatformJoints(){  
  baseJoint[0][0] = -PLATFORM_BASE_RADIUS * cos(degToRad(30) - degToRad(THETA_ANGLE));
  baseJoint[1][0] = -PLATFORM_BASE_RADIUS * cos(degToRad(30) - degToRad(THETA_ANGLE));
  baseJoint[2][0] =  PLATFORM_BASE_RADIUS * sin(degToRad(THETA_ANGLE));
  baseJoint[3][0] =  PLATFORM_BASE_RADIUS * cos(degToRad(30) + degToRad(THETA_ANGLE));
  baseJoint[4][0] =  PLATFORM_BASE_RADIUS * cos(degToRad(30) + degToRad(THETA_ANGLE));
  baseJoint[5][0] =  PLATFORM_BASE_RADIUS * sin(degToRad(THETA_ANGLE));

  baseJoint[0][1] = -PLATFORM_BASE_RADIUS * sin(degToRad(30) - degToRad(THETA_ANGLE));
  baseJoint[1][1] =  PLATFORM_BASE_RADIUS * sin(degToRad(30) - degToRad(THETA_ANGLE));
  baseJoint[2][1] =  PLATFORM_BASE_RADIUS * cos(degToRad(THETA_ANGLE));
  baseJoint[3][1] =  PLATFORM_BASE_RADIUS * sin(degToRad(30) + degToRad(THETA_ANGLE));
  baseJoint[4][1] = -PLATFORM_BASE_RADIUS * sin(degToRad(30) + degToRad(THETA_ANGLE));
  baseJoint[5][1] = -PLATFORM_BASE_RADIUS * cos(degToRad(THETA_ANGLE));

  baseJoint[0][2] =  0;
  baseJoint[1][2] =  0;
  baseJoint[2][2] =  0;
  baseJoint[3][2] =  0;
  baseJoint[4][2] =  0;
  baseJoint[5][2] =  0;

  platformJoint[0][0] = -PLATFORM_TOP_RADIUS * sin(degToRad(30) + degToRad(THETA_R_ANGLE / 2));
  platformJoint[1][0] = -PLATFORM_TOP_RADIUS * sin(degToRad(30) + degToRad(THETA_R_ANGLE / 2));
  platformJoint[2][0] = -PLATFORM_TOP_RADIUS * sin(degToRad(30) - degToRad(THETA_R_ANGLE / 2));
  platformJoint[3][0] =  PLATFORM_TOP_RADIUS * cos(degToRad(THETA_R_ANGLE / 2));
  platformJoint[4][0] =  PLATFORM_TOP_RADIUS * cos(degToRad(THETA_R_ANGLE / 2)); 
  platformJoint[5][0] = -PLATFORM_TOP_RADIUS * sin(degToRad(30) - degToRad(THETA_R_ANGLE / 2));

  platformJoint[0][1] = -PLATFORM_TOP_RADIUS * cos(degToRad(30) + degToRad(THETA_R_ANGLE / 2));
  platformJoint[1][1] =  PLATFORM_TOP_RADIUS * cos(degToRad(30) + degToRad(THETA_R_ANGLE / 2));
  platformJoint[2][1] =  PLATFORM_TOP_RADIUS * cos(degToRad(30) - degToRad(THETA_R_ANGLE / 2));
  platformJoint[3][1] =  PLATFORM_TOP_RADIUS * sin(degToRad(THETA_R_ANGLE / 2));
  platformJoint[4][1] = -PLATFORM_TOP_RADIUS * sin(degToRad(THETA_R_ANGLE / 2));
  platformJoint[5][1] = -PLATFORM_TOP_RADIUS * cos(degToRad(30) - degToRad(THETA_R_ANGLE / 2));

  platformJoint[0][2] =  PLATFORM_HEIGHT_DEFAULT;
  platformJoint[1][2] =  PLATFORM_HEIGHT_DEFAULT;
  platformJoint[2][2] =  PLATFORM_HEIGHT_DEFAULT;
  platformJoint[3][2] =  PLATFORM_HEIGHT_DEFAULT;
  platformJoint[4][2] =  PLATFORM_HEIGHT_DEFAULT;
  platformJoint[5][2] =  PLATFORM_HEIGHT_DEFAULT;
}
    
void setTranslation() {
  translation[0] = ((2*q.x*q.z + 2*q.w*q.y) * sqrt(PLATFORM_HEIGHT_DEFAULT));
  translation[1] = ((2*q.y*q.z - 2*q.w*q.x) * sqrt(PLATFORM_HEIGHT_DEFAULT));
  translation[2] = ((q.w * q.w - q.x*q.x - q.y*q.y + q.z*q.z) * sqrt(PLATFORM_HEIGHT_DEFAULT));
}


void setRotation(float rot[3]) {
  rotation[0] = rot[0];
  rotation[1] = rot[1];
  rotation[2] = rot[2];
}


void getRotationMatrix(float rotationMatrix[3][3]){

  rotationMatrix[0][0] =  q.w * q.w - q.y*q.y - q.z*q.z;
  rotationMatrix[1][0] =  2*q.x*q.y - 2*q.w*q.z;
  rotationMatrix[2][0] =  2*q.x*q.z + 2*q.w*q.y;
  
  rotationMatrix[0][1] =  2*q.x*q.y + 2*q.w*q.z;
  rotationMatrix[1][1] =  q.w * q.w - q.x*q.x - q.z*q.z;
  rotationMatrix[2][1] =  2*q.y*q.z - 2*q.w*q.x;
  
  rotationMatrix[0][2] =  2*q.x*q.z + 2*q.w*q.y;
  rotationMatrix[1][2] =  2*q.y*q.z + 2*q.w*q.x;
  rotationMatrix[2][2] =  q.w * q.w - q.x*q.x - q.y*q.y;
}

void calcLegLength() {
  float rotMatrix[3][3] = {};

  getRotationMatrix(rotMatrix);

  for (int i = 0; i < 6; i++) {
    legLength[i][0] = (rotMatrix[0][0] * platformJoint[i][0]) + (rotMatrix[1][0] * platformJoint[i][1]) + (rotMatrix[2][0] * platformJoint[i][2]);
    legLength[i][1] = (rotMatrix[0][1] * platformJoint[i][0]) + (rotMatrix[1][1] * platformJoint[i][1]) + (rotMatrix[2][1] * platformJoint[i][2]);
    legLength[i][2] = (rotMatrix[0][2] * platformJoint[i][0]) + (rotMatrix[1][2] * platformJoint[i][1]) + (rotMatrix[2][2] * platformJoint[i][2]);
    
    legLength[i][0] += q.y * sqrt(PLATFORM_HEIGHT_DEFAULT) - baseJoint[i][0]; 
    legLength[i][1] += q.z * sqrt(PLATFORM_HEIGHT_DEFAULT) - baseJoint[i][1]; 
    legLength[i][2] += q.x * sqrt(PLATFORM_HEIGHT_DEFAULT) - baseJoint[i][2]; 
  }
}  

void calServoAngle(float servoAngle[6]){
  float lleglength[6];
  for(int i = 0; i < 6; i++){
    lleglength[i] = (legLength[i][0]*legLength[i][0]+legLength[i][1]*legLength[i][1]+legLength[i][2]*legLength[i][2]);
    if(lleglength[i] >= 160*160){
      lleglength[i] = 160*160;
    }else if(lleglength[i] <= 130*130){
      lleglength[i] = 130*130;
    }
    servoAngle[i] = acos((lleglength[i]-LENGTH_SERVO_LEG*LENGTH_SERVO_LEG - LENGTH_SERVO_ARM*LENGTH_SERVO_ARM)/(-2*LENGTH_SERVO_ARM*LENGTH_SERVO_LEG))*180/pi;
    Serial.print("servoAngle [");
    Serial.print(i);
    Serial.print("] : ");


    if(i%2 != 0){
      pwm.setPWM(i*2, 0, pulsewidth(270 - servoAngle[i]/180*135));
      Serial.print((270 - servoAngle[i]/180*135));
    }
    else{
      pwm.setPWM(i*2, 0, pulsewidth(servoAngle[i])/180*135);
      Serial.print((servoAngle[i]/180*135));
    }
    Serial.print("  lleglength [");
    Serial.print(i);
    Serial.print("] : ");
    Serial.println(sqrt(lleglength[i]));
  }
}

