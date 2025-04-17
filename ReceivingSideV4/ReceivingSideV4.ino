// Pins
#define ANALOG_IN A0    // AIN0 (Analog Comparator + input)
#define REF_PIN A1      // AIN1 (Analog Comparator - input)

// Registers
volatile uint16_t highPeriod = 0;  // Timer1 counts (high phase)
volatile uint16_t lowPeriod = 0;   // Timer2 counts (low phase)
volatile uint8_t lastByte = 0;
volatile uint8_t bitPos = 0;
volatile int bits[8] = {0,0,0,0,0,0,0,0};

void setup() {
  Serial.begin(9600);
 
  // Configure Analog Comparator
  ADCSRB = (1 << ACME);     // Enable comparator multiplexer
  ADMUX = (1 << REFS0);     // Use AVcc as reference
  ACSR = (1 << ACIE) | (1 << ACIS1) | (1 << ACIS0); // Interrupt on both edges
 
  // Configure Timer1 (16-bit, high phase)
  TCCR1A = 0;
  TCCR1B = (1 << CS10);     // No prescaler (62.5ns/count)
 
  // Configure Timer2 (8-bit, low phase)
  TCCR2A = 0;
  TCCR2B = (1 << CS20);     // No prescaler (62.5ns/count)
 
  // Set 0.2V reference (10/255 * 5V)
  analogWrite(REF_PIN, 10);
}

ISR(ANALOG_COMP_vect) {
  static bool lastState = false;
 
  if (ACSR & (1 << ACO)) {  // Rising edge (signal > threshold)
    if (!lastState) {
      lowPeriod = TCNT2;    // Capture Timer2 (low phase)
      TCNT2 = 0;            // Reset Timer2
      TCCR1B = (1 << CS10); // Start Timer1
      lastState = true;
    }
  }
  else {                    // Falling edge (signal < threshold)
    if (lastState) {
      highPeriod = TCNT1;   // Capture Timer1 (high phase)
      TCNT1 = 0;            // Reset Timer1
      TCCR2B = (1 << CS20); // Start Timer2
      lastState = false;
     
      // Process measurements immediately
      processPeriods();
    }
  }
}

void processPeriods() {
  // Calculate total period in µs (62.5ns per count)
  uint32_t totalPeriod = (highPeriod + lowPeriod) * 0.0625;
 
  // Classify frequency
  bool isMark = (totalPeriod > 400 && totalPeriod < 450); // 2295Hz
  bool isSpaceF = (totalPeriod > 451 && totalPeriod < 500); // 2125Hz

  // Protocol decoding
  if (!isMark && !isSpaceF) return;
 
  if (isSpaceF && bitPos == 0) { // Start bit
    lastByte = 0;
    bitPos = 1;
  }
  else if (bitPos > 0 && bitPos <= 8) { // Data bits
    if (isMark) lastByte |= (1 << (8 - bitPos));
    bitPos++;
  }
  else if (bitPos > 8 && isMark) { // Stop bit
    Serial.write(lastByte);
    updateBitsArray(lastByte);
    char receivedChar = bitsToChar();
    Serial.print("Received: ");
    Serial.println(receivedChar);
    bitPos = 0;
  }
}

void updateBitsArray(uint8_t byteValue) {
  for (int i = 0; i < 8; i++) {
    bits[i] = (byteValue >> (7 - i)) & 0x01; // Extract bits MSB-first
  }
}

char bitsToChar() {
  char result = 0;
  for (int i = 0; i < 8; i++) {
    result |= (bits[i] << (7 - i)); // Reconstruct byte MSB-first
  }
  return result;
}

void printVoltage() {
  int raw = analogRead(ANALOG_IN);        // Read ADC (0-1023)
  float voltage = raw * (5.0 / 1023.0);       // Convert to volts (5V Arduino)
  Serial.print("Voltage: ");
  Serial.print(voltage, 2);                   // Print with 2 decimal places
  Serial.println("V");
}

void loop() {
  // Optional debug output
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 1000) {
  Serial.print("High: "); Serial.print(highPeriod * 0.0625);
  Serial.print("µs Low: "); Serial.print(lowPeriod * 0.0625);
  Serial.println("µs");
  printVoltage();
  lastPrint = millis();
  }
}
