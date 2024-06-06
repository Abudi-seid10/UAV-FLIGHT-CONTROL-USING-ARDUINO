#include <Wire.h>         // Include the Wire library for I2C communication
#include <math.h>         // Include the math library for mathematical functions

#define PI 3.1415926535897932384626433832795  // Define PI for converting radians to degrees

// Variables to store the rate of rotation and acceleration in x, y, and z axes
float RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ;

// Variables to store calculated angles from accelerometer data
float AngleRoll, AnglePitch;

void gyro_signals(void) {
    // Setup the MPU6050 settings for gyroscope and accelerometer
    Wire.beginTransmission(0x68);
    Wire.write(0x1A);   // Write to the configuration register
    Wire.write(0x05);   // Set the digital low pass filter to a specific setting
    Wire.endTransmission();

    Wire.beginTransmission(0x68);
    Wire.write(0x1C);   // Write to the accelerometer configuration register
    Wire.write(0x10);   // Set accelerometer sensitivity to ±8g
    Wire.endTransmission();

    Wire.beginTransmission(0x68);
    Wire.write(0x3B);   // Request the data starting with accelerometer data
    Wire.endTransmission(false);
    Wire.requestFrom(0x68,6,true);
    int16_t AccXLSB = Wire.read() << 8 | Wire.read();
    int16_t AccYLSB = Wire.read() << 8 | Wire.read();
    int16_t AccZLSB = Wire.read() << 8 | Wire.read();

    // Read gyroscope data
    Wire.beginTransmission(0x68);
    Wire.write(0x1B);   // Gyroscope configuration register
    Wire.write(0x08);   // Set gyroscope sensitivity to ±500 deg/s
    Wire.endTransmission();
    Wire.beginTransmission(0x68);
    Wire.write(0x43);   // Request the data starting with gyroscope data
    Wire.endTransmission();
    Wire.requestFrom(0x68,6);
    int16_t GyroX = Wire.read() << 8 | Wire.read();
    int16_t GyroY = Wire.read() << 8 | Wire.read();
    int16_t GyroZ = Wire.read() << 8 | Wire.read();

    // Process raw data
    RateRoll = (float)GyroX / 65.5;   // Convert to deg/s
    RatePitch = (float)GyroY / 65.5;
    RateYaw = (float)GyroZ / 65.5;
    AccX = (float)AccXLSB / 4096.0 + 0.03;   // Convert to g's
    AccY = (float)AccYLSB / 4096.0 + 0.01;  // Apply offset correction
    AccZ = (float)AccZLSB / 4096.0 - 0.16;

    // Calculate angles from accelerometer data
    AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 180 / PI - 0.55;
    AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 180 / PI - 0.55;
}

void setup() {
    Serial.begin(57600);      // Start serial communication at 57600 baud
    pinMode(13, OUTPUT);      // Set digital pin 13 as an output
    digitalWrite(13, HIGH);   // Set pin 13 high
    Wire.setClock(400000);    // Set I2C clock speed to 400kHz
    Wire.begin();             // Initialize I2C communication
    delay(250);               // Short delay after initializing I2C

    // Wake up the MPU6050
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);         // Write to power management register
    Wire.write(0x00);         // Set register to zero (wakes up the MPU6050)
    Wire.endTransmission();
}

void loop() {
    gyro_signals();           // Read and process sensor data

    // Print acceleration and angle data to the serial monitor in a single line
    // Serial.print("AccX: ");
    // Delay to slow down data output rate
    
      Serial.print(AccX);
    // Serial.print(", AccY: ");
        Serial.print(", ");

    Serial.print(AccY);
    // Serial.print(", AccZ: ");
        Serial.print(", ");

    Serial.print(AccZ);
    // Serial.print(", Roll: ");
        Serial.print(", ");

    Serial.print(AngleRoll);
    // Serial.print(", Pitch: ");
        Serial.print(", ");

    Serial.println(AnglePitch);

    delay(200);             
}
