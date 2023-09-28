#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Select which 'port' M3 or M4
Adafruit_DCMotor* motorL = AFMS.getMotor(3);
Adafruit_DCMotor* motorL = AFMS.getMotor(4);

void setup() {
  // put your setup code here, to run once:
  // start the serial port
  long baudRate = 9600;       // The baudRate for sending & receiving programs must match
  Serial.begin(baudRate);     
  Serial.setTimeout(1); // Set the serial timeout to 1 ms
  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }

  Serial.println("Motor Shield found.");
  // Set the speed to start, from 0 (off) to 255 (max speed)
  motorL->setSpeed(150);
  motorL->run(FORWARD);
  while (Serial.available() - 4 < 0) {}     //wait for data available (any)
  // turn on motor
  motorL->run(RELEASE);
  }

}

void loop() {
  // put your main code here, to run repeatedly:

}
