#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

// ================= ENCODERS =================
#define ENC_LEFT  2
#define ENC_RIGHT 3

// ================= SERVO =================
#define SERVO_PIN A1

// ================= L298N =================
#define ENA 5
#define ENB 6

#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 10

// ================= ULTRASONIC =================
#define TRIG_PIN 12
#define ECHO_PIN 13

MPU6050 mpu;
Servo scanServo;

// ================= ENCODERS =================
volatile long leftPulses = 0;
volatile long rightPulses = 0;

long segmentStartLeft = 0;
long segmentStartRight = 0;

float completedSegmentDistance = 0.0;

// ================= WHEEL DATA =================
const float wheelDiameterCm = 2.5;
const float pulsesPerRevolution = 20.0;

const float wheelCircumferenceCm =
  PI * wheelDiameterCm;

const float cmPerPulse =
  wheelCircumferenceCm / pulsesPerRevolution;

// ================= ROUTE =================
const float targetX = 200.0;
const float targetY = -100.0;

// כיוון הנסיעה בציר Y
// targetY חיובי = שמאלה
// targetY שלילי = ימינה
int yDirection = 1;

// ================= MOTOR SPEED =================
int baseSpeedLeft = 80;
int baseSpeedRight = 120;

const int minMotorSpeed = 75;
const int maxMotorSpeed = 175;

const int reverseSpeed = 110;

const int turnSpeedFast = 130;
const int turnSpeedMedium = 100;
const int turnSpeedSlow = 90;

// ================= PID =================
float Kp = 7.2;
float Ki = 0.635;
float Kd = 3.20;

float KpEncoder = 0.25;

float pidIntegral = 0.0;
float previousAngleError = 0.0;

unsigned long previousPidTime = 0;

const float maxPidIntegral = 100.0;
const float maxCorrection = 45.0;

// ================= TURN =================
const float turnTolerance = 4.0;

/*
  אם בפנייה שמאלה AngleZ עולה:
  השאר 1.

  אם בפנייה שמאלה AngleZ יורד:
  שנה ל--1.
*/
const int LEFT_TURN_SIGN = 1;

// ================= SERVO ANGLES =================
const int SERVO_CENTER = 90;
const int SERVO_RIGHT = 30;
const int SERVO_LEFT = 150;

const unsigned long servoMoveDelayMs = 700;

// ================= OBSTACLE =================
const float obstacleDistanceCm = 16.0;
const float dangerDistanceCm = 6.0;
const float minimumClearSideCm = 28.0;

const float avoidanceSideDistanceCm = 35.0;
const float avoidancePassDistanceCm = 50.0;

const unsigned long turnTimeoutMs = 5000;
const unsigned long driveTimeoutMs = 7000;

const unsigned long obstacleCooldownMs = 1500;
unsigned long lastObstacleTime = 0;

// ================= MPU =================
int16_t ax, ay, az;
int16_t gx, gy, gz;

float gyroZoffset = 0.0;
float angleZ = 0.0;

unsigned long lastGyroTime = 0;

// ================= POSITION =================
float x = 0.0;
float y = 0.0;

float startAngle = 0.0;
float targetTurnAngle = 0.0;

// ================= STATES =================
// 0 = נסיעה בציר X
// 1 = פנייה לכיוון ציר Y
// 2 = נסיעה בציר Y
// 3 = סיום
int state = 0;

bool avoidingObstacle = false;

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Encoders
  pinMode(ENC_LEFT, INPUT_PULLUP);
  pinMode(ENC_RIGHT, INPUT_PULLUP);

  attachInterrupt(
    digitalPinToInterrupt(ENC_LEFT),
    countLeft,
    RISING
  );

  attachInterrupt(
    digitalPinToInterrupt(ENC_RIGHT),
    countRight,
    RISING
  );

  // Motors
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  stopMotors();

  // Ultrasonic
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  digitalWrite(TRIG_PIN, LOW);

  // Servo
  scanServo.attach(SERVO_PIN);
  scanServo.write(SERVO_CENTER);
  delay(700);

  // MPU6050
  Serial.println("Starting MPU6050...");

  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");

    while (1) {
      stopMotors();
    }
  }

  Serial.println("Keep robot still");
  Serial.println("Calibrating gyro...");

  delay(1000);

  calibrateGyro();

  angleZ = 0.0;
  startAngle = 0.0;

  // קביעת כיוון ציר Y
  if (targetY >= 0.0) {
    yDirection = 1;

    // Y חיובי = פנייה שמאלה
    targetTurnAngle =
      startAngle +
      LEFT_TURN_SIGN * 90.0;
  }
  else {
    yDirection = -1;

    // Y שלילי = פנייה ימינה
    targetTurnAngle =
      startAngle -
      LEFT_TURN_SIGN * 90.0;
  }

  lastGyroTime = micros();
  previousPidTime = millis();

  resetSegmentEncoders();
  resetPID();

  Serial.println();
  Serial.println("ROUTE START");

  Serial.print("Target X = ");
  Serial.println(targetX);

  Serial.print("Target Y = ");
  Serial.println(targetY);

  Serial.print("Target turn angle = ");
  Serial.println(targetTurnAngle);

  if (yDirection > 0) {
    Serial.println("Y direction: LEFT");
  }
  else {
    Serial.println("Y direction: RIGHT");
  }
}

// =====================================================
// LOOP
// =====================================================
void loop() {
  updateAngle();

  switch (state) {

    // =================================================
    // נסיעה בציר X
    // =================================================
    case 0: {
      float frontDistance =
        readDistanceFiltered();

      bool cooldownFinished =
        millis() - lastObstacleTime >
        obstacleCooldownMs;

      if (
        cooldownFinished &&
        !avoidingObstacle &&
        frontDistance > 2.0 &&
        frontDistance <= obstacleDistanceCm
      ) {
        saveCurrentSegmentProgress();

        bool success =
          scanAndAvoidObstacle(startAngle);

        resetSegmentEncoders();
        resetPID();

        if (!success) {
          stopMotors();
          delay(800);
          break;
        }
      }

      driveStraightPID(startAngle);

      x = getTotalSegmentDistance();
      y = 0.0;

      if (x >= absoluteFloat(targetX)) {
        stopMotors();

        x = targetX;

        Serial.println("Finished X");

        delay(500);

        completedSegmentDistance = 0.0;

        resetSegmentEncoders();
        resetPID();

        state = 1;
      }

      break;
    }

    // =================================================
    // פנייה לכיוון ציר Y
    // =================================================
    case 1: {
      bool finishedTurn =
        turnToAngle(targetTurnAngle);

      if (finishedTurn) {
        stopMotors();

        if (yDirection > 0) {
          Serial.println("Finished LEFT turn");
        }
        else {
          Serial.println("Finished RIGHT turn");
        }

        delay(500);

        completedSegmentDistance = 0.0;

        resetSegmentEncoders();
        resetPID();

        state = 2;
      }

      break;
    }

    // =================================================
    // נסיעה בציר Y
    // =================================================
    case 2: {
      float frontDistance =
        readDistanceFiltered();

      bool cooldownFinished =
        millis() - lastObstacleTime >
        obstacleCooldownMs;

      if (
        cooldownFinished &&
        !avoidingObstacle &&
        frontDistance > 2.0 &&
        frontDistance <= obstacleDistanceCm
      ) {
        saveCurrentSegmentProgress();

        bool success =
          scanAndAvoidObstacle(targetTurnAngle);

        resetSegmentEncoders();
        resetPID();

        if (!success) {
          stopMotors();
          delay(800);
          break;
        }
      }

      driveStraightPID(targetTurnAngle);

      float traveledY =
        getTotalSegmentDistance();

      x = targetX;

      // מוסיפים סימן לפי הכיוון
      y = yDirection * traveledY;

      // בודקים מרחק מוחלט ולא y >= targetY
      if (
        traveledY >=
        absoluteFloat(targetY)
      ) {
        stopMotors();

        x = targetX;
        y = targetY;

        state = 3;

        printFinalData();
      }

      break;
    }

    // =================================================
    // סיום
    // =================================================
    case 3:
      stopMotors();
      return;
  }

  printData();

  delay(20);
}

// =====================================================
// MPU CALIBRATION
// =====================================================
void calibrateGyro() {
  long sumGz = 0;

  const int samples = 600;

  for (int i = 0; i < samples; i++) {
    mpu.getMotion6(
      &ax, &ay, &az,
      &gx, &gy, &gz
    );

    sumGz += gz;

    delay(3);
  }

  gyroZoffset =
    sumGz / (float)samples;

  Serial.print("Gyro offset = ");
  Serial.println(gyroZoffset);
}

// =====================================================
// UPDATE ANGLE
// =====================================================
void updateAngle() {
  unsigned long currentTime = micros();

  float dt =
    (currentTime - lastGyroTime) /
    1000000.0;

  lastGyroTime = currentTime;

  if (dt <= 0.0 || dt > 0.1) {
    dt = 0.01;
  }

  mpu.getMotion6(
    &ax, &ay, &az,
    &gx, &gy, &gz
  );

  float gyroZrate =
    (gz - gyroZoffset) / 131.0;

  angleZ += gyroZrate * dt;
}

// =====================================================
// PID RESET
// =====================================================
void resetPID() {
  pidIntegral = 0.0;
  previousAngleError = 0.0;
  previousPidTime = millis();
}

// =====================================================
// DRIVE STRAIGHT WITH PID
// =====================================================
void driveStraightPID(float desiredAngle) {
  long L;
  long R;

  getEncoderPulses(L, R);

  long deltaL =
    L - segmentStartLeft;

  long deltaR =
    R - segmentStartRight;

  float angleError =
    angleZ - desiredAngle;

  unsigned long currentTime =
    millis();

  float dt =
    (currentTime - previousPidTime) /
    1000.0;

  if (dt <= 0.0 || dt > 0.2) {
    dt = 0.02;
  }

  previousPidTime = currentTime;

  float proportional =
    Kp * angleError;

  pidIntegral +=
    angleError * dt;

  pidIntegral = constrain(
    pidIntegral,
    -maxPidIntegral,
    maxPidIntegral
  );

  float integral =
    Ki * pidIntegral;

  float derivativeError =
    (angleError - previousAngleError) /
    dt;

  float derivative =
    Kd * derivativeError;

  previousAngleError =
    angleError;

  long encoderError =
    deltaL - deltaR;

  float encoderCorrection =
    KpEncoder * encoderError;

  float pidOutput =
      proportional
    + integral
    + derivative;

  float totalCorrection =
      pidOutput
    - encoderCorrection;

  totalCorrection = constrain(
    totalCorrection,
    -maxCorrection,
    maxCorrection
  );

  int leftSpeed =
    baseSpeedLeft +
    (int)totalCorrection;

  int rightSpeed =
    baseSpeedRight -
    (int)totalCorrection;

  leftSpeed = constrain(
    leftSpeed,
    minMotorSpeed,
    maxMotorSpeed
  );

  rightSpeed = constrain(
    rightSpeed,
    minMotorSpeed,
    maxMotorSpeed
  );

  driveForward(
    leftSpeed,
    rightSpeed
  );
}

// =====================================================
// AUTOMATIC TURN TO ANGLE
// בוחר לבד ימינה או שמאלה
// =====================================================
bool turnToAngle(float desiredAngle) {
  float angleError =
    desiredAngle - angleZ;

  float absError =
    absoluteFloat(angleError);

  if (absError <= turnTolerance) {
    stopMotors();
    return true;
  }

  int speedValue =
    chooseTurnSpeed(absError);

  /*
    אם AngleZ גדל בפנייה שמאלה:
    שגיאה חיובית = שמאלה
    שגיאה שלילית = ימינה

    LEFT_TURN_SIGN מאפשר להפוך את הכיוון
    במקרה שהג'יירו מחובר הפוך.
  */
  if (
    angleError * LEFT_TURN_SIGN > 0.0
  ) {
    turnLeft(speedValue);
  }
  else {
    turnRight(speedValue);
  }

  return false;
}

// =====================================================
// TURN SPEED
// =====================================================
int chooseTurnSpeed(float angleError) {
  if (angleError < 8.0) {
    return turnSpeedSlow;
  }

  if (angleError < 20.0) {
    return turnSpeedMedium;
  }

  return turnSpeedFast;
}

// =====================================================
// ULTRASONIC
// =====================================================
float readUltrasonicDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);

  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration =
    pulseIn(
      ECHO_PIN,
      HIGH,
      30000UL
    );

  if (duration == 0) {
    return 400.0;
  }

  float distance =
    duration * 0.0343 / 2.0;

  if (
    distance < 2.0 ||
    distance > 400.0
  ) {
    return 400.0;
  }

  return distance;
}

// =====================================================
// FILTERED DISTANCE
// =====================================================
float readDistanceFiltered() {
  float d1 =
    readUltrasonicDistance();

  delay(30);

  float d2 =
    readUltrasonicDistance();

  delay(30);

  float d3 =
    readUltrasonicDistance();

  return medianOfThree(
    d1,
    d2,
    d3
  );
}

// =====================================================
// MEDIAN
// =====================================================
float medianOfThree(
  float a,
  float b,
  float c
) {
  if (a > b) {
    float temp = a;
    a = b;
    b = temp;
  }

  if (b > c) {
    float temp = b;
    b = c;
    c = temp;
  }

  if (a > b) {
    float temp = a;
    a = b;
    b = temp;
  }

  return b;
}

// =====================================================
// MOVE SERVO AND READ
// =====================================================
float lookAtAngle(int servoAngle) {
  stopMotors();

  Serial.print("Servo moving to ");
  Serial.println(servoAngle);

  scanServo.write(servoAngle);

  unsigned long startTime =
    millis();

  while (
    millis() - startTime <
    servoMoveDelayMs
  ) {
    updateAngle();
    delay(5);
  }

  // מתעלמים מהקריאה הראשונה
  readUltrasonicDistance();
  delay(60);

  float distance =
    readDistanceFiltered();

  Serial.print("Angle=");
  Serial.print(servoAngle);

  Serial.print(" Distance=");
  Serial.println(distance);

  return distance;
}

// =====================================================
// FULL SCAN
// =====================================================
void scanEnvironment(
  float &rightDistance,
  float &centerDistance,
  float &leftDistance
) {
  stopMotors();

  Serial.println("SCAN START");

  scanServo.write(SERVO_CENTER);
  delay(500);

  rightDistance =
    lookAtAngle(SERVO_RIGHT);

  centerDistance =
    lookAtAngle(SERVO_CENTER);

  leftDistance =
    lookAtAngle(SERVO_LEFT);

  scanServo.write(SERVO_CENTER);
  delay(500);

  Serial.print("RIGHT=");
  Serial.print(rightDistance);

  Serial.print(" CENTER=");
  Serial.print(centerDistance);

  Serial.print(" LEFT=");
  Serial.println(leftDistance);
}

// =====================================================
// SCAN AND AVOID
// =====================================================
bool scanAndAvoidObstacle(
  float originalAngle
) {
  if (avoidingObstacle) {
    return false;
  }

  avoidingObstacle = true;

  stopMotors();
  resetPID();

  delay(300);

  float frontDistance =
    readDistanceFiltered();

  if (
    frontDistance <= dangerDistanceCm
  ) {
    Serial.println("Reversing");

    unsigned long reverseStart =
      millis();

    while (
      millis() - reverseStart < 500
    ) {
      updateAngle();

      driveBackward(
        reverseSpeed,
        reverseSpeed
      );

      delay(5);
    }

    stopMotors();
    delay(300);
  }

  float rightDistance;
  float centerDistance;
  float leftDistance;

  scanEnvironment(
    rightDistance,
    centerDistance,
    leftDistance
  );

  bool rightClear =
    rightDistance >=
    minimumClearSideCm;

  bool leftClear =
    leftDistance >=
    minimumClearSideCm;

  bool success = false;

  if (!rightClear && !leftClear) {
    Serial.println(
      "Both sides blocked"
    );

    unsigned long reverseStart =
      millis();

    while (
      millis() - reverseStart < 700
    ) {
      updateAngle();

      driveBackward(
        reverseSpeed,
        reverseSpeed
      );

      delay(5);
    }

    stopMotors();

    scanServo.write(SERVO_CENTER);

    avoidingObstacle = false;
    lastObstacleTime = millis();

    return false;
  }

  if (
    rightClear &&
    (
      !leftClear ||
      rightDistance >= leftDistance
    )
  ) {
    Serial.println("Choosing RIGHT");

    success =
      avoidToRight(originalAngle);
  }
  else {
    Serial.println("Choosing LEFT");

    success =
      avoidToLeft(originalAngle);
  }

  scanServo.write(SERVO_CENTER);
  delay(500);

  resetPID();
  resetSegmentEncoders();

  avoidingObstacle = false;
  lastObstacleTime = millis();

  return success;
}

// =====================================================
// AVOID RIGHT
// =====================================================
bool avoidToRight(float originalAngle) {
  float rightHeading =
    originalAngle -
    LEFT_TURN_SIGN * 90.0;

  float leftHeading =
    originalAngle +
    LEFT_TURN_SIGN * 90.0;

  if (!turnRightBlocking(rightHeading)) {
    return false;
  }

  if (
    !driveDistanceBlocking(
      avoidanceSideDistanceCm,
      rightHeading
    )
  ) {
    return false;
  }

  if (!turnLeftBlocking(originalAngle)) {
    return false;
  }

  if (
    !driveDistanceBlocking(
      avoidancePassDistanceCm,
      originalAngle
    )
  ) {
    return false;
  }

  if (!turnLeftBlocking(leftHeading)) {
    return false;
  }

  if (
    !driveDistanceBlocking(
      avoidanceSideDistanceCm,
      leftHeading
    )
  ) {
    return false;
  }

  if (!turnRightBlocking(originalAngle)) {
    return false;
  }

  return true;
}

// =====================================================
// AVOID LEFT
// =====================================================
bool avoidToLeft(float originalAngle) {
  float leftHeading =
    originalAngle +
    LEFT_TURN_SIGN * 90.0;

  float rightHeading =
    originalAngle -
    LEFT_TURN_SIGN * 90.0;

  if (!turnLeftBlocking(leftHeading)) {
    return false;
  }

  if (
    !driveDistanceBlocking(
      avoidanceSideDistanceCm,
      leftHeading
    )
  ) {
    return false;
  }

  if (!turnRightBlocking(originalAngle)) {
    return false;
  }

  if (
    !driveDistanceBlocking(
      avoidancePassDistanceCm,
      originalAngle
    )
  ) {
    return false;
  }

  if (!turnRightBlocking(rightHeading)) {
    return false;
  }

  if (
    !driveDistanceBlocking(
      avoidanceSideDistanceCm,
      rightHeading
    )
  ) {
    return false;
  }

  if (!turnLeftBlocking(originalAngle)) {
    return false;
  }

  return true;
}

// =====================================================
// LEFT BLOCKING TURN
// =====================================================
bool turnLeftBlocking(float desiredAngle) {
  unsigned long startTime =
    millis();

  while (
    absoluteFloat(
      desiredAngle - angleZ
    ) > turnTolerance
  ) {
    updateAngle();

    float error =
      absoluteFloat(
        desiredAngle - angleZ
      );

    turnLeft(
      chooseTurnSpeed(error)
    );

    if (
      millis() - startTime >
      turnTimeoutMs
    ) {
      stopMotors();

      Serial.println(
        "Left turn timeout"
      );

      return false;
    }

    delay(5);
  }

  stopMotors();
  delay(200);

  return true;
}

// =====================================================
// RIGHT BLOCKING TURN
// =====================================================
bool turnRightBlocking(float desiredAngle) {
  unsigned long startTime =
    millis();

  while (
    absoluteFloat(
      desiredAngle - angleZ
    ) > turnTolerance
  ) {
    updateAngle();

    float error =
      absoluteFloat(
        desiredAngle - angleZ
      );

    turnRight(
      chooseTurnSpeed(error)
    );

    if (
      millis() - startTime >
      turnTimeoutMs
    ) {
      stopMotors();

      Serial.println(
        "Right turn timeout"
      );

      return false;
    }

    delay(5);
  }

  stopMotors();
  delay(200);

  return true;
}

// =====================================================
// DRIVE FIXED DISTANCE
// =====================================================
bool driveDistanceBlocking(
  float targetDistance,
  float desiredAngle
) {
  resetSegmentEncoders();
  resetPID();

  scanServo.write(SERVO_CENTER);

  unsigned long startTime =
    millis();

  while (
    getSegmentDistance() <
    targetDistance
  ) {
    updateAngle();

    float frontDistance =
      readDistanceFiltered();

    if (
      frontDistance > 2.0 &&
      frontDistance <= dangerDistanceCm
    ) {
      stopMotors();

      Serial.println(
        "Blocked during avoidance"
      );

      return false;
    }

    driveStraightPID(desiredAngle);

    if (
      millis() - startTime >
      driveTimeoutMs
    ) {
      stopMotors();

      Serial.println(
        "Drive timeout"
      );

      return false;
    }

    delay(10);
  }

  stopMotors();
  delay(200);

  return true;
}

// =====================================================
// SAVE PROGRESS
// =====================================================
void saveCurrentSegmentProgress() {
  completedSegmentDistance +=
    getSegmentDistance();

  stopMotors();

  Serial.print("Saved distance=");
  Serial.println(
    completedSegmentDistance
  );
}

// =====================================================
// TOTAL SEGMENT DISTANCE
// =====================================================
float getTotalSegmentDistance() {
  return
    completedSegmentDistance +
    getSegmentDistance();
}

// =====================================================
// DISTANCE
// =====================================================
float getSegmentDistance() {
  long L;
  long R;

  getEncoderPulses(L, R);

  long deltaL =
    L - segmentStartLeft;

  long deltaR =
    R - segmentStartRight;

  float averagePulses =
    (deltaL + deltaR) / 2.0;

  return
    averagePulses * cmPerPulse;
}

// =====================================================
// RESET SEGMENT
// =====================================================
void resetSegmentEncoders() {
  noInterrupts();

  segmentStartLeft =
    leftPulses;

  segmentStartRight =
    rightPulses;

  interrupts();
}

// =====================================================
// READ ENCODERS
// =====================================================
void getEncoderPulses(
  long &L,
  long &R
) {
  noInterrupts();

  L = leftPulses;
  R = rightPulses;

  interrupts();
}

// =====================================================
// INTERRUPTS
// =====================================================
void countLeft() {
  leftPulses++;
}

void countRight() {
  rightPulses++;
}

// =====================================================
// MOTORS
// =====================================================
void driveForward(
  int leftSpeed,
  int rightSpeed
) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);
}

void driveBackward(
  int leftSpeed,
  int rightSpeed
) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);
}

void turnLeft(int speedValue) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, speedValue);
  analogWrite(ENB, speedValue);
}

void turnRight(int speedValue) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, speedValue);
  analogWrite(ENB, speedValue);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// =====================================================
// ABS
// =====================================================
float absoluteFloat(float value) {
  if (value < 0.0) {
    return -value;
  }

  return value;
}

// =====================================================
// SERIAL
// =====================================================
void printData() {
  static unsigned long lastPrintTime = 0;

  if (
    millis() - lastPrintTime < 250
  ) {
    return;
  }

  lastPrintTime = millis();

  long L;
  long R;

  getEncoderPulses(L, R);

  Serial.print("State=");
  Serial.print(state);

  Serial.print(" X=");
  Serial.print(x);

  Serial.print(" Y=");
  Serial.print(y);

  Serial.print(" TargetAngle=");
  Serial.print(targetTurnAngle);

  Serial.print(" Angle=");
  Serial.print(angleZ);

  Serial.print(" L=");
  Serial.print(L);

  Serial.print(" R=");
  Serial.println(R);
}

void printFinalData() {
  Serial.println();
  Serial.println(
    "========== FINISHED =========="
  );

  Serial.print("Final X=");
  Serial.println(x);

  Serial.print("Final Y=");
  Serial.println(y);

  Serial.print("Final Angle=");
  Serial.println(angleZ);
}