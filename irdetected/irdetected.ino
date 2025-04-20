void setup() {
  Serial.begin(9600);
  pinMode(2, INPUT);
}

void loop() {
  // Measure the length of the high pulse in microseconds
  unsigned long highTime = pulseIn(2, HIGH);
  // Measure the length of the low pulse in microseconds
  unsigned long lowTime = pulseIn(2, LOW);
  unsigned long period = highTime + lowTime;

  if (period > 0) {
    float frequency = 1000000.0 / period;
    Serial.print("Frequency: ");
    Serial.print(frequency);
    Serial.println(" Hz");
  }
  delay(100);
}
