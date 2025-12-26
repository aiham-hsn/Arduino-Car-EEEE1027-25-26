// Input pins to drive the motors
const uint8_t IN1 = 13;
const uint8_t IN2 = 12;
const uint8_t IN3 = 18;  // marked as pin A4 on the Arduino
const uint8_t IN4 = 19;  // marked as pin A5 on the Arduino
//const uint8_t IN3 = 1;  // DISCONNECT PINS FROM ARDUINO WHEN UPLOADING SKETCH
//const uint8_t IN4 = 0;  // DISCONNECT PINS FROM ARDUINO WHEN UPLOADING SKETCH

// Motor enable pins
const uint8_t ENA = 11;
const uint8_t ENB = 3;

// Preset speeds that can be used to drive the motor at
const uint8_t TOP_GEAR = 255;
const uint8_t UPPER_GEAR = 128;
const uint8_t REV_UPPER_GEAR = 170;
const uint8_t NORMAL_GEAR = 80;
const uint8_t LOWER_GEAR = 50;

unsigned long start_time = 0;

void commandRotate(uint8_t speed) {
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
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

void setup() {
  // Set the L298N's input pins as outputs from the Arduino
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  delay(1000);
  start_time = millis();
}

void loop() {
  if ((millis() - start_time) < 6000) {
    commandRotate(REV_UPPER_GEAR);
  }
  else { stop(); }
}
