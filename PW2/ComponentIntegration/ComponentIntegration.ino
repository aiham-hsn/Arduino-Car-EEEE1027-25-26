#include <LiquidCrystal.h>

//Function declarations
void printDist(float dist, uint8_t col, uint8_t row);
void printTime(unsigned long msecs, uint8_t col, uint8_t row);
void forward(uint8_t speed);
void backward(uint8_t speed);
void left(uint8_t speed);
void right(uint8_t speed);
void stop();

// LCD setup pins
const uint8_t RS = 8;
const uint8_t EN = 9;
const uint8_t D4 = 4;
const uint8_t D5 = 5;
const uint8_t D6 = 6;
const uint8_t D7 = 7;

// Input pins to drive the motors
const uint8_t IN1 = 13;
const uint8_t IN2 = 12;
const uint8_t IN3 = 18; // marked as pin A5 on the Arduino
const uint8_t IN4 = 19; // marked as pin A4 on the Arduino

// Motor enable pins
const uint8_t ENA = 11;
const uint8_t ENB = 3;

// Preset speeds that can be used to drive the motor at
const uint8_t MAX_SPEED = 255;
const uint8_t UPPER_SPEED = 150;
const uint8_t NORMAL_SPEED = 100;
const uint8_t LOWER_SPEED = 50;

// Rotary Encoder pins
//const uint8_t ENC_L = 12;  // Not in use as pin 12 cannot be used for digitalPinToInterrupt
const uint8_t ENC_R = 2;

//IR Sensor pins
const uint8_t IR_L = 16;  // marked as pin A2 on the Arduino
const uint8_t IR_R = 17;  // marked as pin A3 on the Arduino

//IR Sensor threshold values
const uint8_t IRT_L = 255;  // temporary value, actual value to be determined empirically later
const uint8_t IRT_R = 255;  // temporary value, actual value to be determined empirically later

// Variables related to counting pulses received from the rotary encoders
//volatile unsigned int countL = 0;  // Not in use, see comment on ENC_L
volatile unsigned int countR = 0;

// Define interrupt functions to be used to count the pulses from the encoders
//void count_left() { countL++; }  // Not in use, see comment on ENC_L
void count_rght() { countR++; }

// Variables related to time and state keeping
unsigned long DURATION = 3 * 1000;  // Converted to milliseconds
unsigned long start_time = 0;
unsigned long elapsed_time = 0;
bool is_moving = false;

// Variables related to printing and calculating distance travelled
float distance = 0;
const float CIRCUMFRNC = 26.748671; // Circumference in cm ; Calculated from a diameter measurement of 67mm
const uint8_t PPR = 40;  // Pulses per revolution, counted from the number of slits in the encoder wheel.
                         // TODO: empirically measure the pulses generated per revolution
                         // PPR empirically determined to be 40


// Setup the LCD
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

void setup() {
  // Set encoder digital out pins as inputs to the Arduino
  //pinMode(ENC_L, INPUT);
  pinMode(ENC_R, INPUT);

  // Set the L298N's input pins as outputs from the Arduino
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Attach interrupts for counting the pulses
  //attachInterrupt(digitalPinToInterrupt(ENC_L), count_left, FALLING);  // Not in use, see comment on ENC_L
  attachInterrupt(digitalPinToInterrupt(ENC_R), count_rght, FALLING);

  // Init LCD
  lcd.begin(16, 2);

  // Display run start message
  lcd.setCursor(0, 0);
  lcd.print("Beginning run...");
  delay(750);

  // Start run & record start time
  lcd.clear();
  start_time = millis();
}

void loop() {
  // Calculate elapsed time
  elapsed_time = millis() - start_time;

  // Calculate distance travelled
  //distance = ((float)((countL + countR) / 2) / (float)PPR) * CIRCUMFRNC;
  distance = ((float)countR / (float)PPR) * CIRCUMFRNC;

  // Only move if elapsed time is less than 10s
  if (elapsed_time <= DURATION) {
    // Move forwards
    forward(UPPER_SPEED);
    // Print time taken and distance travelled
    printTime(elapsed_time, 0, 0);
    printDist(distance, 0, 1);
  } else {
    stop();
  }
}

void printDist(float dist, uint8_t col, uint8_t row) {
  lcd.setCursor(col, row);
  lcd.print("Dist (cm): ");
  lcd.print(dist, 1); // print to 2 DP
}
void printTime(unsigned long msecs, uint8_t col, uint8_t row) {
  const double secs = (double)msecs / (double)1000;
  lcd.setCursor(col, row);
  lcd.print("Time (s): ");
  lcd.print(secs, 1);
}

void forward(uint8_t speed) {
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  if (is_moving == false) { is_moving = true; }
}
void backward(uint8_t speed) {
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  if (is_moving == false) { is_moving = true; }
}
void left(uint8_t speed) {
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  if (is_moving == false) { is_moving = true; }
}
void right(uint8_t speed) {
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  if (is_moving == false) { is_moving = true; }
}
void stop() {
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  if (is_moving == true) { is_moving = false; }
}

