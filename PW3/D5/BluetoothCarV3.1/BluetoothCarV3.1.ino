#include "Headers.h"

#define FORWARD 'F'
#define BACKWARD 'B'
#define LEFT 'L'
#define RIGHT 'R'
#define CIRCLE 'C'
#define CROSS 'X'
#define TRIANGLE 'T'
#define SQUARE 'S'
#define START 'A'
#define PAUSE 'P'

// Input pins to drive the motors
//const uint8_t IN1 = 13;
//const uint8_t IN2 = 12;
const uint8_t IN1 = 18;    // marked as pin A4 on the Arduino
const uint8_t IN2 = 19;    // marked as pin A5 on the Arduino
//const uint8_t IN3 = 18;  // marked as pin A4 on the Arduino
//const uint8_t IN4 = 19;  // marked as pin A5 on the Arduino
//const uint8_t IN3 = 1;   // DISCONNECT PINS FROM ARDUINO WHEN UPLOADING SKETCH
//const uint8_t IN4 = 0;   // DISCONNECT PINS FROM ARDUINO WHEN UPLOADING SKETCH
const uint8_t IN3 = 16;    // marked as pin A2 on the Arduino
const uint8_t IN4 = 17;    // marked as pin A3 on the Arduino

// Motor enable pins
const uint8_t ENA = 11;
const uint8_t ENB = 3;

// Preset speeds that can be used to drive the motor at
const uint8_t TOP_GEAR = 255;
const uint8_t UPPER_GEAR = 128;
const uint8_t REV_UPPER_GEAR = 170;
const uint8_t NORMAL_GEAR = 90;
const uint8_t LOWER_GEAR = 50;

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    delay(20);
  }

  // Set the L298N's input pins as outputs from the Arduino
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  stop();
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    Serial.println(command);
    executeCommand(command);
  }
}

void executeCommand(char command) {
  switch (command) {
    case FORWARD:
      // Perform action for moving forward
      forward(UPPER_GEAR);
      break;
    case BACKWARD:
      // Perform action for moving backward
      backward(UPPER_GEAR);
      break;
    case LEFT:
      // Perform action for turning left
      left(UPPER_GEAR);
      break;
    case RIGHT:
      // Perform action for turning right
      right(UPPER_GEAR);
      break;
    case CIRCLE:
      // Perform action for circle
      stop();
      break;
    case CROSS:
      // Perform action for immediate stop or crossing
      backward(UPPER_GEAR);
      break;
    case TRIANGLE:
      // Perform action for toggling a state (e.g., LED on/off)
      forward(UPPER_GEAR);
      break;
    case SQUARE:
      // Perform action for retrieving and sending status information
      break;
    case START:
      // Perform action for starting a process or operation
      break;
    case PAUSE:
      // Perform action for pausing a process or operation
      break;
    default:
      // Invalid command received
      stop();
      break;
  }
}

void forward(uint8_t speed) {
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}
void backward(uint8_t speed) {
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}
void left(uint8_t speed) {
  analogWrite(ENA, speed);
  analogWrite(ENB, REV_UPPER_GEAR);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}
void right(uint8_t speed) {
  analogWrite(ENA, speed);
  analogWrite(ENB, REV_UPPER_GEAR);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}
void stop() {
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
