#include <SoftwareWire.h>
#include <Servo.h>
#include <Wire.h>

// Define PI for converting radians to degrees
#define PI 3.1415926535897932384626433832795

// Define ESC signals
#define MAX_SIGNAL 1040
#define MIN_SIGNAL 1075

// Use SoftwareWire instead of Wire
SoftwareWire SWire(A0, A1); // Set SDA to A0 and SCL to A1

// Variables to store control signals
int throttle = 1000;
int roll = 0;
int pitch = 0;
int yaw = 0;
bool arm = false;
bool kill = false;

// Motor pin definitions
const int motor1Pin = 9;
const int motor2Pin = 10;
const int motor3Pin = 11;
const int motor4Pin = 6;

// Motor speeds
int motor1Speed = MIN_SIGNAL;
int motor2Speed = MIN_SIGNAL;
int motor3Speed = MIN_SIGNAL;
int motor4Speed = MIN_SIGNAL;

// Target motor speeds
int targetMotor1Speed = MIN_SIGNAL;
int targetMotor2Speed = MIN_SIGNAL;
int targetMotor3Speed = MIN_SIGNAL;
int targetMotor4Speed = MIN_SIGNAL;

// Servo objects for motors
Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

// Variables to store the rate of rotation and acceleration in x, y, and z axes
float RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ;

// Variables to store calculated angles from accelerometer data
float AngleRoll, AnglePitch;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  SWire.begin(); // Initialize SoftwareWire for MPU6050
  Wire.begin(8); // Initialize I2C as slave with address 8
  Wire.onReceive(receiveEvent); // Register receive event

  Serial.println("Arduino ready to receive control signals");

  // Initialize motor pins
  pinMode(motor1Pin, OUTPUT);
  pinMode(motor2Pin, OUTPUT);
  pinMode(motor3Pin, OUTPUT);
  pinMode(motor4Pin, OUTPUT);

 // Attach motors to their respective pins
  motor1.attach(motor1Pin);
  motor2.attach(motor2Pin);
  motor3.attach(motor3Pin);
  motor4.attach(motor4Pin);

  // Calibrate ESCs
  calibrateESCs();

  // Arm motors initially
  armMotors(false);

  // Initialize MPU6050
  setupMPU6050();
}

void loop() {
  // Update gyro signals
  readMPU6050();

  // Maintain angle within ±1 degree
  maintainAngle(0, 0); // Target angles can be adjusted as needed

  // Update motor speeds based on control signals
  updateMotors();

  // Incrementally adjust motor speeds
  incrementMotorSpeeds();

  // Print motor speeds for debugging
  Serial.print("Motor1: ");
  Serial.print(motor1Speed);
  Serial.print(", Motor2: ");
  Serial.print(motor2Speed);
  Serial.print(", Motor3: ");
  Serial.print(motor3Speed);
  Serial.print(", Motor4: ");
  Serial.print(motor4Speed);
  Serial.print(", Throttle: ");
  Serial.print(throttle);
  Serial.print(", Roll: ");
  Serial.print(roll);
  Serial.print(", Pitch: ");
  Serial.print(pitch);
  Serial.print(", Yaw: ");
  Serial.print(yaw);
  Serial.print(", Arm: ");
  Serial.print(arm);
  Serial.print(", Roll angle: ");
  Serial.print(AngleRoll);
  Serial.print(", Pitch angle: ");
  Serial.println(AnglePitch);
}

void setupMPU6050() {
  SWire.beginTransmission(0x68); // MPU6050 I2C address
  SWire.write(0x6B);             // PWR_MGMT_1 register
  SWire.write(0);                // Set to zero (wakes up the MPU-6050)
  SWire.endTransmission(true);

  SWire.beginTransmission(0x68);
  SWire.write(0x1A);             // Configuration register
  SWire.write(0x05);             // Set DLPF_CFG to 5
  SWire.endTransmission(true);

  SWire.beginTransmission(0x68);
  SWire.write(0x1B);             // Gyroscope configuration register
  SWire.write(0x08);             // Set gyroscope sensitivity to ±500 deg/s
  SWire.endTransmission(true);

  SWire.beginTransmission(0x68);
  SWire.write(0x1C);             // Accelerometer configuration register
  SWire.write(0x10);             // Set accelerometer sensitivity to ±8g
  SWire.endTransmission(true);
}

void readMPU6050() {
  SWire.beginTransmission(0x68);
  SWire.write(0x3B);   // Starting register for accelerometer data
  SWire.endTransmission(false);
  SWire.requestFrom(0x68, 14, true); // Request a total of 14 registers

  // Read accelerometer data
  AccX = SWire.read() << 8 | SWire.read();
  AccY = SWire.read() << 8 | SWire.read();
  AccZ = SWire.read() << 8 | SWire.read();
  SWire.read(); // Read temperature registers (ignore)
  SWire.read();

  // Read gyroscope data
  RateRoll = SWire.read() << 8 | SWire.read();
  RatePitch = SWire.read() << 8 | SWire.read();
  RateYaw = SWire.read() << 8 | SWire.read();

  // Convert raw data to meaningful values
  RateRoll /= 65.5;  // Convert to deg/s
  RatePitch /= 65.5;
  RateYaw /= 65.5;
  AccX = (float)AccX / 4096.0; // Convert to g's
  AccY = (float)AccY / 4096.0 + 0.03; // Apply offset correction
  AccZ = (float)AccZ / 4096.0 - 0.16;

  // Calculate angles from accelerometer data
  AngleRoll = atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * 180 / PI - 2.06;
  AnglePitch = atan2(AccX, sqrt(AccY * AccY + AccZ * AccZ)) * 180 / PI + 2.70;
}

void maintainAngle(float targetRoll, float targetPitch) {
  if (AngleRoll > targetRoll + 20) {
    roll -= 5;  // Adjust the roll control signal to decrease roll
  } else if (AngleRoll < targetRoll - 20) {
    roll += 5;  // Adjust the roll control signal to increase roll
  }

  if (AnglePitch > targetPitch + 20) {
    pitch -= 5;  // Adjust the pitch control signal to decrease pitch
  } else if (AnglePitch < targetPitch - 20) {
    pitch += 5;  // Adjust the pitch control signal to increase pitch
  }

  // Constrain the control signals to avoid excessive values
  roll = constrain(roll, -100, 100);
  pitch = constrain(pitch, -100, 100);

  delay(20);
}

void parseControlSignals() {
  if (Wire.available() >= 6) { // Adjusted to read 6 bytes of data
    int highByte = Wire.read();
    int lowByte = Wire.read();
    throttle = (highByte << 8) | lowByte; // Combine high and low bytes for throttle
    roll = Wire.read();
    pitch = Wire.read();
    yaw = Wire.read();
    int status = Wire.read();
    arm = (status == 1);
    
    // Debugging prints
    Serial.print("Received data: ");
    Serial.print(throttle);
    Serial.print(" ");
    Serial.print(roll);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.print(yaw);
    Serial.print(" ");
    Serial.println(status);
  }
}

void updateMotors() {
  if (throttle == 1000) {
    targetMotor1Speed = throttle;
    targetMotor2Speed = throttle;
    targetMotor3Speed = throttle;
    targetMotor4Speed = throttle;
  } else {
    if (arm) {
      targetMotor1Speed = throttle + pitch - roll + yaw;
      targetMotor2Speed = throttle + pitch + roll - yaw;
      targetMotor3Speed = throttle - pitch + roll + yaw;
      targetMotor4Speed = throttle - pitch - roll - yaw;

      // Ensure motor speeds are within acceptable range
      targetMotor1Speed = constrain(targetMotor1Speed, 1000, 2000);
      targetMotor2Speed = constrain(targetMotor2Speed, 1000, 2000);
      targetMotor3Speed = constrain(targetMotor3Speed, 1000, 2000);
      targetMotor4Speed = constrain(targetMotor4Speed, 1000, 2000);
    } else {
      armMotors(false);
    }
  }
}

void incrementMotorSpeeds() {
  smoothTransition(motor1Speed, targetMotor1Speed);
  smoothTransition(motor2Speed, targetMotor2Speed);
  smoothTransition(motor3Speed, targetMotor3Speed);
  smoothTransition(motor4Speed, targetMotor4Speed);
}

void smoothTransition(int &currentValue, int targetValue) {
  if (currentValue != targetValue) {
    if (currentValue < targetValue) {
      currentValue = currentValue + 5;
    } else if (currentValue > targetValue) {
      currentValue = currentValue - 20;
    }
  }
  motor1.writeMicroseconds(motor1Speed);
  motor2.writeMicroseconds(motor2Speed);
  motor3.writeMicroseconds(motor3Speed);
  motor4.writeMicroseconds(motor4Speed);
  delay(10); // Adjust the delay to control the speed of the transition
}

void armMotors(bool state) {
  if (!state) {
    Serial.println("Drone KILLED!");
    motor1.writeMicroseconds(1000); // Send minimum signal
    motor2.writeMicroseconds(1000);
    motor3.writeMicroseconds(1000);
    motor4.writeMicroseconds(1000);
  } else {
    Serial.println("Drone ARMED!");
    motor1.writeMicroseconds(1130);
    motor2.writeMicroseconds(1130);
    motor3.writeMicroseconds(1130);
    motor4.writeMicroseconds(1130);
  }
}

void calibrateESCs() {
  Serial.println("Calibrating ESCs...");
  Serial.println("Turn on power source and wait for calibration to complete.");



  // Wait for 2 seconds before starting the calibration
  delay(2000);

  Serial.println("ESCs calibrated");
}

void receiveEvent(int howMany) {
  parseControlSignals();
}
