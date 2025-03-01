/*
 * Photodiode will pick up the 2125Hz signal emitting from the
 * IRled and take it as a current. Arduino will read the current
 * and determine the voltage off it.
 */

int analogPin = A0;

void setup() {
  Serial.begin(9600); // 9600 bits per second communication
  pinMode(analogPin, INPUT);
}

void loop() {
  // Read the analog value from Pin A0
  int analogValue = analogRead(analogPin);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  float voltage = analogValue * (5.0 / 1023.0);
  if (voltage > 2.5) // 10-bit ADC with 512 as the threshold
  {
    // Do something when the signal is high (1)
    Serial.print(voltage);
    Serial.print(": HIGH");
    Serial.println();
  } else
  {
    // Do something when the signal is low (0)
    Serial.print(voltage);
    Serial.print(": LOW");
    Serial.println();
  }
}
