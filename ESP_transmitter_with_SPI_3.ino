#include <Wire.h>
#include <BluetoothSerial.h>

// Define the device name and PIN
const char *pin = "1234"; // Change this to a more secure PIN
String device_name = "ESP32-Drone-Controller";

BluetoothSerial SerialBT;

// Variables for drone control
int throttle = 1100;
int roll = 0;
int pitch = 0;
int yaw = 0;
int status = 0;

// Aux variables
String numericPart = "";
char codeReceived;

void setup() {
    // Init serial
    Serial.begin(9600); // For debugging
    SerialBT.begin(device_name); // Bluetooth device name
    SerialBT.setTimeout(5);
    Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str());
#ifdef USE_PIN
    SerialBT.setPin(pin);
    Serial.println("Using PIN");
#endif

    Wire.begin(18, 19); // Initialize I2C with custom pins (SDA: 18, SCL: 19)
    Serial.println("Initialization complete");
}

void loop() {
    // Read control signals from Bluetooth
    while (SerialBT.available() > 0) {
        String line = SerialBT.readStringUntil('\n');
        numericPart = "";
        for (int i = 0; i < line.length(); i++) {
            int character = line[i];
            if (isDigit(character)) {
                numericPart += (char)character;
            } else if (character != '\n') {
                codeReceived = character;
            } else {
                break;
            }
        }
        processCommand(codeReceived, numericPart);
    }
    delay(50);
}

int tempT, tempY, tempR, tempP;

void processCommand(char command, String value) {
    int intValue = value.toInt();
    switch (command) {
        case 'T': // Throttle
            tempT = constrain(intValue, 0, 40);
            throttle = map(tempT, 0, 40, 1130, 1700);
            break;
        case 'Y': // Yaw
            tempY = constrain(intValue, 0, 20);
            yaw = map(tempY, 0, 20, -100, 100);
            break;
        case 'R': // Roll (from joystick X-axis)
            tempR = constrain(intValue, 0, 20);
            roll = map(tempR, 0, 20, -100, 100);
            break;
        case 'P': // Pitch (from joystick Y-axis)
            tempP = constrain(intValue, 0, 20);
            pitch = map(tempP, 0, 20, -100, 100);
            break;
        case 'A': // Arm
            status = 1;
            throttle = 1130;
            break;
        case 'K': // Kill
            status = 0;
            throttle = 1000;
            break;
    }
    sendControlSignals();
}

void sendControlSignals() {
    Serial.print("Sending data: ");
    Serial.print(throttle);
    Serial.print(" ");
    Serial.print(roll);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.print(yaw);
    Serial.print(" ");
    Serial.println(status);

    Wire.beginTransmission(8); // Start I2C communication with slave device address 8
    Wire.write(throttle >> 8); // Send high byte of throttle
    Wire.write(throttle & 0xFF); // Send low byte of throttle
    Wire.write(roll);
    Wire.write(pitch);
    Wire.write(yaw);
    Wire.write(status);
    Wire.endTransmission();
}
