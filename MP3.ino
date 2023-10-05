#include <Adafruit_MotorShield.h>
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Select which 'port' M3 or M4
Adafruit_DCMotor* motorL = AFMS.getMotor(4);
Adafruit_DCMotor* motorR = AFMS.getMotor(3);

#define lsensor_pin 0
#define rsensor_pin 1

uint16_t sensor_floor = 780;
uint32_t time;
uint32_t loop_time = 0;
uint32_t LOOP_INTERVAL = 500;
int displacement = 0;
uint16_t base_speed = 50;
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
  AFMS.begin();
  Serial.println("Motor Shield Ready");
  // Set the speed to start, from 0 (off) to 255 (max speed)
  motorL->setSpeed(50);
  motorL->run(RELEASE);
  motorR->setSpeed(50);
  motorR->run(RELEASE);
  
  while (Serial.available() - 4 < 0) {}     //wait for data available (any)
  // turn on motor
  loop_time = millis();
}


void loop() {
  // Serial.print(analogRead(0));
  // Serial.print(",");
  // Serial.println(analogRead(1));
  time = millis();
  if (Serial.available() - 1 < 0 && Serial.read() == ':'){
    while (Serial.available() - 1 < 0) {}     //wait for data available
    int num = Serial.read();
    base_speed = num * 10; // set speed to typed 1 int value
  }
  // put your main code here, to run repeatedly:
  displacement = calc_error();
  Serial.println(displacement);

  motor_run_left(displacement/5);
  motor_run_right(displacement/5);
}


void motor_run_left(int error){
  motorL->setSpeed((uint8_t)(base_speed+error));
  motorL->run(BACKWARD);
}

void motor_run_right(int error){
  motorR->setSpeed((uint8_t)(base_speed-error));
  motorR->run(FORWARD);
}

int calc_error(){
  // todo - remap sensor inputs to range
  double lsensor = analogRead(lsensor_pin);
  double rsensor = analogRead(rsensor_pin);
  lsensor -= 803;
  rsensor -= 822;
  return lsensor - rsensor;
}