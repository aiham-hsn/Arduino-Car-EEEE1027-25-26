// Rotary Encoder pins
const byte ENC = 2;

volatile unsigned int count = 0;

void counter() { count++; }

void setup()
{
  // Init serial comms
  Serial.begin(9600);

  // Set encoder digital out pins as inputs to the Arduino
  pinMode(ENC, INPUT);

  // Attach interrupts for counting the pulses
  attachInterrupt(digitalPinToInterrupt(ENC), counter, FALLING);
}

void loop()
{
  // Print number of pulses counted to the serial console
  Serial.print("Pulse count: ");
  Serial.println(count);
}
