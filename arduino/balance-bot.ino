#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <PID_v1.h>

#define IMU_INT_PIN 2

#define MOTOR_ENA_PIN 5
#define MOTOR_IN1_PIN 6
#define MOTOR_IN2_PIN 7

#define MOTOR_ENB_PIN 10
#define MOTOR_IN3_PIN 8
#define MOTOR_IN4_PIN 9

bool setupSuccess = true;

MPU6050 imu;
volatile bool imuHasData = false;
uint16_t imuDataSize;
uint8_t imuFifoBuffer[256];
Quaternion imuQuaternion;
VectorFloat imuGravity;
VectorInt16 imuAngularVelocity;
float imuYpr[3];
unsigned long imuIdx = 0;
const int16_t imuXGyroOffset = 86;
const int16_t imuYGyroOffset = 67;
const int16_t imuZGyroOffset = 29;
const int16_t imuZAccelOffset = 969;

double balancedAngle = 87.16;
double Kp = 46;
double Ki = 0;
double Kd = 0.15;
// double Kp = 4.6;
// double Ki = 0;
// double Kd = 0.026;
double angle;
double motorValue;
PID pid(&angle, &motorValue, &balancedAngle, Kp, Ki, Kd, DIRECT);
const double maxAngle = 40;

void imuInterrupt() {
  imuHasData = true;
}

bool connectImu() {
  Serial.print(F("Connecting IMU..."));

  imu.initialize();

  bool imuDmpError = imu.dmpInitialize();
  if (imuDmpError) {
    Serial.println(F("DMP error"));
    return false;
  }

  imu.setXGyroOffset(imuXGyroOffset);
  imu.setYGyroOffset(imuYGyroOffset);
  imu.setZGyroOffset(imuZGyroOffset);
  imu.setZAccelOffset(imuZAccelOffset);

  imu.setDMPEnabled(true);

  attachInterrupt(digitalPinToInterrupt(IMU_INT_PIN), imuInterrupt, RISING);

  imu.resetFIFO();

  Serial.println(F("Connected"));

  return true;
}

bool connectPID() {
  Serial.print(F("Connecting PID..."));

  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(2);
  // pid.SetOutputLimits(-5, 5);
  // pid.SetOutputLimits(-255, 255);
  pid.SetOutputLimits(-15.9, 15.9);

  Serial.println(F("Connected"));

  return true;
}

bool connectMotor() {
  Serial.print(F("Connecting Motor..."));

  pinMode(MOTOR_ENA_PIN, OUTPUT);
  pinMode(MOTOR_ENB_PIN, OUTPUT);
  pinMode(MOTOR_IN1_PIN, OUTPUT);
  pinMode(MOTOR_IN2_PIN, OUTPUT);
  pinMode(MOTOR_IN3_PIN, OUTPUT);
  pinMode(MOTOR_IN4_PIN, OUTPUT);

  analogWrite(MOTOR_ENA_PIN, 0);
  analogWrite(MOTOR_ENB_PIN, 0);
  digitalWrite(MOTOR_IN1_PIN, LOW);
  digitalWrite(MOTOR_IN2_PIN, LOW);
  digitalWrite(MOTOR_IN3_PIN, LOW);
  digitalWrite(MOTOR_IN4_PIN, LOW);

  Serial.println(F("Connected"));

  return true;
}

bool getImuAngle() {
  if (!imu.dmpGetCurrentFIFOPacket(imuFifoBuffer)) {
    return false;
  }

  imuHasData = false;

  imu.dmpGetQuaternion(&imuQuaternion, imuFifoBuffer);
  imu.dmpGetGravity(&imuGravity, &imuQuaternion);
  imu.dmpGetYawPitchRoll(imuYpr, &imuQuaternion, &imuGravity);

  angle = imuYpr[1] * 180 / M_PI;
  angle += (angle > 0 ? -180 : 180);

  return true;
}

void getMotorValue() {
  pid.Compute();

  if (imuIdx % 1 == 0) {
    Serial.print(-250);
    Serial.print(F("\t"));
    Serial.print(250);
    Serial.print(F("\t"));
    Serial.print(angle);
    Serial.print(F("\t"));
    // Serial.println(motorValue);
    Serial.println(motorValue*motorValue);
  }
  imuIdx += 1;
}

void driveMotor() {
  if (abs(angle - balancedAngle) > maxAngle || motorValue==0) {
    motorStop();
    return;
  }

  if (motorValue > 0) {
    motorForward();
  } else {
    motorBackward();
  }
}

void motorStop() {
  analogWrite(MOTOR_ENA_PIN, 0);
  analogWrite(MOTOR_ENB_PIN, 0);
  digitalWrite(MOTOR_IN1_PIN, LOW);
  digitalWrite(MOTOR_IN2_PIN, LOW);
  digitalWrite(MOTOR_IN3_PIN, LOW);
  digitalWrite(MOTOR_IN4_PIN, LOW);
}
void motorForward() {
  analogWrite(MOTOR_ENA_PIN, motorValue*motorValue);
  analogWrite(MOTOR_ENB_PIN, motorValue*motorValue);
  digitalWrite(MOTOR_IN1_PIN, HIGH);
  digitalWrite(MOTOR_IN2_PIN, LOW);
  digitalWrite(MOTOR_IN3_PIN, HIGH);
  digitalWrite(MOTOR_IN4_PIN, LOW);
}
void motorBackward() {
  analogWrite(MOTOR_ENA_PIN, motorValue*motorValue);
  analogWrite(MOTOR_ENB_PIN, motorValue*motorValue);
  digitalWrite(MOTOR_IN1_PIN, LOW);
  digitalWrite(MOTOR_IN2_PIN, HIGH);
  digitalWrite(MOTOR_IN3_PIN, LOW);
  digitalWrite(MOTOR_IN4_PIN, HIGH);
}


void setup() {
  Serial.begin(115200);

  setupSuccess &= connectImu();

  setupSuccess &= connectPID();

  setupSuccess &= connectMotor();

  Serial.print(F("Setup succeed: "));
  Serial.println(setupSuccess);
}


void loop() {
  if (!setupSuccess) { return; }

  if (!imuHasData) { return; }

  if (!getImuAngle()) { return; }

  getMotorValue();

  driveMotor();
}
