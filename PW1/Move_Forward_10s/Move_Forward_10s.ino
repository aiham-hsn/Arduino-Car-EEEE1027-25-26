// Input pins to drive the motors
const byte IN1 = 12;
const byte IN2 = 11;
const byte IN3 = 3;
const byte IN4 = 2;

// Motor enable pins
const byte ENA = 15; // marked as pin A1 on the Arduino
const byte ENB = 16; // marked as pin A2 on the Arduino

// Variables related to motor run duration
unsigned long startTime = 0;
const word DURATION = 10000; // 10 * 1000

void setup()
{
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);

  // Record start time
  startTime = millis();
}

void loop()
{
  // Only move if elapsed time is less than 10s
  if ((millis() - startTime) < DURATION) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
}
