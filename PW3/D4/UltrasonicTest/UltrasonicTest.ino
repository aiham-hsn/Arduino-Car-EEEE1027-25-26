// Function declarations
unsigned long readDuration(uint8_t trigger, uint8_t echo);
void forward(uint8_t speed);
void right(uint8_t speed);
void stop();

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
const uint8_t NORMAL_GEAR = 80;
const uint8_t LOWER_GEAR = 50;

// Ultrasonic sensor pins
const uint8_t ECHO = 12;  // Used to trigger the ultrasonic pulse
const uint8_t TRIG = 13;  // Used to listen to the return echo of the ultrasonic pulse

unsigned long duration = 0;
double durationMs = 0;
float distanceCm = 0;

void setup() {
  // Attempt to start a serial connection to the Arduino IDE
  Serial.begin(115200);
  while (!Serial) {
    delay(20); // will pause Zero, Leonardo, etc until serial console opens
  }

  // Set the L298N's input pins as outputs from the Arduino
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Set the ultrasonic sensor's pins accordingly
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
}

void loop() {
  duration = readDuration(TRIG, ECHO);
  durationMs = (double)duration / (double)1000;
  distanceCm = ((float)duration / 2.0) * 0.034;  // The speed of sound is approximately 0.034 centimeters per microsecond at room temperature and pressure

  Serial.print("duration:");
  Serial.print(duration);
  Serial.print(",");
  Serial.print("durationMs:");
  Serial.print(durationMs);
  Serial.print(",");
  Serial.print("distanceCm:");
  Serial.print(distanceCm);
  Serial.println("");

  delay(50);
}

// Returns a non-zero value if the ultrasonic sensor detects an echo of the ultrasonic pulse it generates
// The non-zero value is the time taken in microseconds to receive the echo of the pulse after the pulse has been sent
unsigned long readDuration(uint8_t trigger, uint8_t echo) {
  // Command the ultrasonic sensor to send an ultrasonic pulse
  digitalWrite(trigger, LOW);
  delayMicroseconds(4);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);

  // Wait for the sensor to detect the echo of the pulse which sets the corresponding pin to the HIGH state
  // Record how long said pin is in the high state for
  // If a response is not detected within 38.01 milliseconds it is determined that there is nothing within 400cm of the sensor
  // Which means the duration will be zero
  duration = pulseIn(echo, HIGH, 38010);
}

void forward(uint8_t speed) {
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
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
