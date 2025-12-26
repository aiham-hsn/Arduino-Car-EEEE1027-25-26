// Input pins to drive the motors
const byte IN1 = 12;
const byte IN2 = 11;
const byte IN3 = 3;
const byte IN4 = 2;

// Motor enable pins
const byte ENA = 15; // marked as pin A1 on the Arduino
const byte ENB = 16; // marked as pin A2 on the Arduino

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
}

void loop()
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}
