#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Select which 'port' M3 or M4
Adafruit_DCMotor* motorL = AFMS.getMotor(3);
Adafruit_DCMotor* motorR = AFMS.getMotor(4);

uint8_t lsensor_pin = 0;
uint8_t rsensor_pin = 1;
uint16_t sensor_floor = 780;
uint32_t time;
uint32_t loop_time = 0;
uint32_t LOOP_INTERVAL = 500;
uint16_t displacement = 0;
uint16_t angle_adjustment = 0;
uint16_t zero_ptr = 0;
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
  Serial.println("Motor Shield found.");
  // Set the speed to start, from 0 (off) to 255 (max speed)
  motorL->setSpeed(50);
  motorL->run(RELEASE);
  motorR->setSpeed(50);
  motorR->run(RELEASE);
  // motorL->run(RELEASE);
  // motorR->run(RELEASE);
  
  while (Serial.available() - 4 < 0) {}     //wait for data available (any)
  // turn on motor
  while (Serial.available() - 4 < 0) {}     //wait for data available (any)
  loop_time = millis();

  // PID(&displacement, &angle_adjustment, &zero_ptr, Kp, Ki, Kd, DIRECT) 
}


uint8_t turn_scaling = 100;
void loop() {
  Serial.print(analogRead(0));
  Serial.print(",");
  Serial.println(analogRead(1));
  time = millis();
  // put your main code here, to run repeatedly:
  displacement = calc_error();

  motor_run_left(displacement);
  motor_run_right(displacement);
  while (time - loop_time > LOOP_INTERVAL) {
    time = millis();
  }
  loop_time = millis();
}

uint16_t base_speed = 50;
void motor_run_left(uint16_t error){
  motorL->setSpeed(base_speed+error);
  motorL->run(BACKWARD);
}

void motor_run_right(uint16_t error){
  motorR->setSpeed(base_speed-error);
  motorR->run(FORWARD);
}

uint16_t calc_error(){
  // todo - remap sensor inputs to range
  uint16_t lsensor = analogRead(lsensor_pin);
  uint16_t rsensor = analogRead(rsensor_pin);
  lsensor -= sensor_floor;
  rsensor -= sensor_floor;
  return lsensor - rsensor;
}