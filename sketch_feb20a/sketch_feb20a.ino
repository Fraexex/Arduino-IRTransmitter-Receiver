volatile bool transmitting = false;
volatile uint8_t currentByte = 0;
volatile uint8_t bitCount = 0;
volatile const char* message = NULL;

void setup() {
  // ... (previous setup code) ...

  // Configure Timer1 for carrier frequency
  // Configure Timer2 for bit timing
}

ISR(TIMER1_COMPA_vect) {
  if (transmitting) {
    PORTB ^= (1 << PORTB1);  // Toggle LED for carrier
  }
}

ISR(TIMER2_COMPA_vect) {
  if (transmitting) {
    // Handle bit transmission logic here
    // Update currentByte, bitCount, and message pointer as needed
    // Turn carrier on/off based on current bit
  }
}

void transmitMessage(const char* msg) {
  message = msg;
  currentByte = *message;
  bitCount = 0;
  transmitting = true;
  // Enable Timer2 interrupt
}

void loop() {
  transmitMessage("Hello World!");
  while (transmitting) {
    // Wait for transmission to complete
  }
  Serial.println(1);
  delay(5000);
}
