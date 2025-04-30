#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "I2Cdev.h"
#include "MPU6050.h"

// Motor Pins
const int ledPin = 13;
const int ENA = 3;    // PWM Motor A
const int IN1 = 4;    // Direction Motor A
const int IN2 = 5;

const int ENB = 6;    // PWM Motor B
const int IN3 = 7;    // Direction Motor B
const int IN4 = 8;

// Angle Variables
float targetAngle = 0.0;   // Target is upright position (0°)
float angleX = 0.0;

// Calibration Variables
float gyroXOffset = 0.0;
float accelXOffset = 0.0;

// PID Variables
float Kp = 1.0;  // Proportional gain
float Ki = 0.1;  // Integral gain
float Kd = 0.5;  // Derivative gain

float previousError = 0.0;
float integralError = 0.0;
float derivativeError = 0.0;
float pidOutput = 0.0;

// Timer Variables for hold time
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
unsigned long elapsedTime = 0;
bool isUpright = false;  // To check if robot is within upright range
unsigned long holdTime = 5000;  // 5 seconds to turn off the LED after manual hold

// LCD Setup (16x2)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// MPU6050 Setup
MPU6050 mpu;

// Complementary filter coefficient
float alpha = 0.96;  // Can be adjusted based on performance

void setup() {
  Serial.begin(115200);
  lcd.begin(16, 2);
  lcd.setBacklight(LOW);  // Turn off backlight initially
  lcd.setCursor(0, 0);
  lcd.print("Upright Time:");

  // Initialize Motor Pins
  pinMode(ledPin, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  digitalWrite(ledPin, LOW);
  stopMotors(); // Ensure motors are off initially

  // Initialize MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }

  // Gyro Calibration
  Serial.println("Calibrating gyro...");
  long gyroXSum = 0;
  int16_t gx;
  for (int i = 0; i < 500; i++) {
    mpu.getRotation(&gx, nullptr, nullptr);
    gyroXSum += gx;
    delay(2);
  }
  gyroXOffset = (float)gyroXSum / 500.0;
  Serial.print("Gyro X Offset: ");
  Serial.println(gyroXOffset);

  // Set vertical as 0 angle
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  accelXOffset = atan2(ay, az) * 180.0 / PI;
  angleX = 0;

  previousMillis = millis();  // Initialize time tracking
}

void loop() {
  // Get sensor data
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  float correctedGyroX = gx - gyroXOffset;

  unsigned long currentMillis = millis();
  float dt = (currentMillis - previousMillis) / 1000.0;  // Time difference in seconds
  previousMillis = currentMillis;

  // Get tilt angle using accelerometer and gyro
  float accelAngleX = atan2(ay, az) * 180.0 / PI - accelXOffset;
  float gyroAngleChange = (correctedGyroX / 131.0) * dt;
  angleX = alpha * (angleX + gyroAngleChange) + (1 - alpha) * accelAngleX;

  // Display angle data
  Serial.print("Angle X: ");
  Serial.println(angleX);

  // LED logic
  if ((angleX > 5 && angleX < 80) || (angleX < -5 && angleX > -80)) {
    digitalWrite(ledPin, HIGH);
    isUpright = true;
    startTimer(); // Start timing when the robot is upright
  } else {
    digitalWrite(ledPin, LOW);
    isUpright = false;
    stopTimer(); // Stop timing when the robot falls
  }

  // Calculate PID
  float error = targetAngle - angleX;
  integralError += error * dt;
  derivativeError = (error - previousError) / dt;

  pidOutput = Kp * error + Ki * integralError + Kd * derivativeError;

  // Update previous error
  previousError = error;

  // Motor control logic with PID correction
  int motorSpeed = map(abs(pidOutput), 0, 255, 100, 255); // Motor speed based on PID output
  motorSpeed = constrain(motorSpeed, 0, 255); // Ensure motor speed is within 0-255

  if (angleX > 5) {
    moveBackward(motorSpeed);  // Tilted forward, move backward to correct
  } else if (angleX < -5) {
    moveForward(motorSpeed);   // Tilted backward, move forward to correct
  } else {
    stopMotors();              // Balanced
  }

  // Display time on LCD
  if (isUpright) {
    lcd.setCursor(0, 1);
    lcd.print("Time: ");
    lcd.print(elapsedTime / 1000); // Show time in seconds
  }
}

void startTimer() {
  if (elapsedTime == 0) {  // Only start the timer once
    previousMillis = millis();  // Reset the timer when the robot is upright
  }
}

void stopTimer() {
  if (elapsedTime >= holdTime) {
    lcd.setCursor(0, 1);
    lcd.print("Hold time over!");
    digitalWrite(ledPin, LOW);  // Turn off the LED after holding for 5 seconds
  } else if (elapsedTime > 0) {
    // If the robot falls, display the final time
    lcd.setCursor(0, 1);
    lcd.print("Fallen! Time: ");
    lcd.print(elapsedTime / 1000);  // Show time in seconds
  }
}

void moveForward(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, speed);  // Speed control (adjust as needed)
  analogWrite(ENB, speed);
}

void moveBackward(int speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}
