#include <Servo.h>

#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1100
int DELAY = 1000;

Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

void setup() {
  Serial.begin(9600);
  Serial.println("Don't forget to subscribe!");
  Serial.println("ELECTRONOOBS ESC calibration...");
  Serial.println(" ");
  delay(1500);
  Serial.println("Program begin...");
  delay(1000);
  Serial.println("This program will start the ESCs.");

  motor1.attach(9);
  motor2.attach(10);
  motor3.attach(11);
  motor4.attach(6);

  Serial.print("Now writing maximum output: ("); Serial.print(MAX_SIGNAL); Serial.print(" us in this case)"); Serial.print("\n");
  Serial.println("Turn on power source, then wait 2 seconds and press any key.");
  motor1.writeMicroseconds(MAX_SIGNAL);
  motor2.writeMicroseconds(MAX_SIGNAL);
  motor3.writeMicroseconds(MAX_SIGNAL);
  motor4.writeMicroseconds(MAX_SIGNAL);

  // Wait for input
  while (!Serial.available());
  Serial.read();

  // Send min output
  Serial.println("\n");
  Serial.println("\n");
  Serial.print("Sending minimum output: ("); Serial.print(MIN_SIGNAL); Serial.print(" us in this case)"); Serial.print("\n");
  motor1.writeMicroseconds(MIN_SIGNAL);
  motor2.writeMicroseconds(MIN_SIGNAL);
  motor3.writeMicroseconds(MIN_SIGNAL);
  motor4.writeMicroseconds(MIN_SIGNAL);
  Serial.println("The ESCs are calibrated");
  Serial.println("----");
  Serial.println("Now, type a value between 1000 and 2000 and press enter");
  Serial.println("and the motors will start rotating.");
  Serial.println("Send 1000 to stop the motors and 2000 for full throttle");

}

void loop() {
  
    int DELAY = Serial.parseInt();
    if (DELAY > 999)
    {
      motor1.writeMicroseconds(DELAY);
      motor2.writeMicroseconds(DELAY);
      motor3.writeMicroseconds(DELAY);
      motor4.writeMicroseconds(DELAY);
      float SPEED = (DELAY - 1000) / 10.0;
      Serial.print("\n");
      Serial.println("Motor speed:"); Serial.print("  "); Serial.print(SPEED); Serial.print("%");
    }

}
