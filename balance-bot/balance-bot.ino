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
VectorInt16 imuAcceleration;
VectorInt16 imuAccWithoutGravity;
VectorInt16 imuAccWorld;
unsigned long imuIdx = 0;
const int16_t imuXGyroOffset = 80;
const int16_t imuYGyroOffset = 58;
const int16_t imuZGyroOffset = 17;
const int16_t imuXAccelOffset = -825;
const int16_t imuYAccelOffset = -461;
const int16_t imuZAccelOffset = 964;

int sampleMs = 2;

double absoluteZero = 0;

double balancedPitch = 178;
const double maxPitchAngle = 40;
const double survivingPitch = 4;
double pitchKp = 50;
double pitchKi = 1520 / sampleMs;
double pitchKd = 0.1125 * sampleMs;
double pitchKpSurviving = 100;
double pitchKiSurviving = 0;
double pitchKdSurviving = 0.04 * sampleMs;
double pitchAngle;
double motorValue;
const double maxMotorValue = 255;
PID pitchPID(&pitchAngle, &motorValue, &absoluteZero, pitchKp, pitchKi, pitchKd, DIRECT);

double balancedYaw = 0;
double yawKp = 5;
double yawKi = 0;
double yawKd = 0.04 * sampleMs;
double yawAngle;
double motorCorrection;
const double maxMotorCorrection = 60;
PID yawPID(&yawAngle, &motorCorrection, &absoluteZero, yawKp, yawKi, yawKd, DIRECT);


void imuInterrupt() {
  imuHasData = true;
}

bool initImu() {
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
  imu.setXAccelOffset(imuXAccelOffset);
  imu.setYAccelOffset(imuYAccelOffset);
  imu.setZAccelOffset(imuZAccelOffset);

  imu.setDMPEnabled(true);

  attachInterrupt(digitalPinToInterrupt(IMU_INT_PIN), imuInterrupt, RISING);

  imu.resetFIFO();

  delay(8000);

  while (!imuHasData) {}
  while (!getRobotState()) {}
  balancedYaw = yawAngle;

  Serial.println(F("Connected"));

  return true;
}

bool initPID() {
  Serial.print(F("Connecting PID..."));

  pitchPID.SetMode(AUTOMATIC);
  pitchPID.SetSampleTime(sampleMs);
  pitchPID.SetOutputLimits(-maxMotorValue, maxMotorValue);

  yawPID.SetMode(AUTOMATIC);
  yawPID.SetSampleTime(sampleMs);
  yawPID.SetOutputLimits(-maxMotorCorrection, maxMotorCorrection);

  Serial.println(F("Connected"));

  return true;
}

bool initMotor() {
  Serial.print(F("Connecting Motor..."));

  pinMode(MOTOR_ENA_PIN, OUTPUT);
  pinMode(MOTOR_ENB_PIN, OUTPUT);
  pinMode(MOTOR_IN1_PIN, OUTPUT);
  pinMode(MOTOR_IN2_PIN, OUTPUT);
  pinMode(MOTOR_IN3_PIN, OUTPUT);
  pinMode(MOTOR_IN4_PIN, OUTPUT);

  motorStop();

  Serial.println(F("Connected"));

  return true;
}

double unwrapAngle(float angle, double balanced) {
  double a = angle * 180 / M_PI;
  a -= balanced;
  if (a < -180) {
    a += 360;
  } else if (a > 180) {
    a -= 360;
  }
  return a;
}

bool getRobotState() {
  if (!imu.dmpGetCurrentFIFOPacket(imuFifoBuffer)) {
    return false;
  }

  imuHasData = false;

  imu.dmpGetQuaternion(&imuQuaternion, imuFifoBuffer);
  imu.dmpGetGravity(&imuGravity, &imuQuaternion);
  imu.dmpGetYawPitchRoll(imuYpr, &imuQuaternion, &imuGravity);

  imu.dmpGetAccel(&imuAcceleration, imuFifoBuffer);
  imu.dmpGetLinearAccel(&imuAccWithoutGravity, &imuAcceleration, &imuGravity);
  imu.dmpGetLinearAccelInWorld(&imuAccWorld, &imuAccWithoutGravity, &imuQuaternion);

  pitchAngle = unwrapAngle(imuYpr[1], balancedPitch);

  yawAngle = unwrapAngle(imuYpr[0], balancedYaw);

  return true;
}

void getFeedbackValue() {
  if (abs(pitchAngle) > survivingPitch) {
    pitchPID.SetTunings(pitchKpSurviving, pitchKiSurviving, pitchKdSurviving);
  } else {
    pitchPID.SetTunings(pitchKp, pitchKi, pitchKd);
  }

  pitchPID.Compute();
  yawPID.Compute();

  if (imuIdx % 1 == 0) {
    Serial.print(-250);
    Serial.print(F("\t"));
    Serial.print(250);

    Serial.print(F("\t"));
    Serial.print(pitchAngle);
    Serial.print(F("\t"));
    Serial.println(motorValue);

    // Serial.print(F("\t"));
    // Serial.print(yawAngle);
    // Serial.print(F("\t"));
    // Serial.println(motorCorrection);
  }
  imuIdx += 1;
}


void motorStop() {
  motorMove(0, MOTOR_ENA_PIN, MOTOR_IN1_PIN, MOTOR_IN2_PIN);
  motorMove(0, MOTOR_ENB_PIN, MOTOR_IN3_PIN, MOTOR_IN4_PIN);
}

void motorMove(double value, uint8_t enPin, uint8_t inPin1, uint8_t inPin2) {
  if (value > 0) {
    analogWrite(enPin, value);
    digitalWrite(inPin1, HIGH);
    digitalWrite(inPin2, LOW);
    return;
  }
  if (value < 0) {
    analogWrite(enPin, -value);
    digitalWrite(inPin1, LOW);
    digitalWrite(inPin2, HIGH);
    return;
  }
  analogWrite(enPin, 0);
  digitalWrite(inPin1, LOW);
  digitalWrite(inPin2, LOW);
}

void takeAction() {
  if (abs(pitchAngle) > maxPitchAngle) {
    motorStop();
    return;
  }

  if (abs(motorValue) > maxMotorValue * 0.36) {
    motorMove(motorValue, MOTOR_ENA_PIN, MOTOR_IN1_PIN, MOTOR_IN2_PIN);
    motorMove(motorValue, MOTOR_ENB_PIN, MOTOR_IN3_PIN, MOTOR_IN4_PIN);
  } else {
    motorMove(motorValue + motorCorrection, MOTOR_ENA_PIN, MOTOR_IN1_PIN, MOTOR_IN2_PIN);
    motorMove(motorValue - motorCorrection, MOTOR_ENB_PIN, MOTOR_IN3_PIN, MOTOR_IN4_PIN);
  }
}


void setup() {
  Serial.begin(115200);

  setupSuccess &= initImu();

  setupSuccess &= initPID();

  setupSuccess &= initMotor();

  Serial.print(F("Setup succeed: "));
  Serial.println(setupSuccess);
}


void loop() {
  if (!setupSuccess) { return; }

  if (!imuHasData) { return; }

  if (!getRobotState()) { return; }

  getFeedbackValue();

  takeAction();
}
