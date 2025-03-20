#define IR_LED_PIN 9 // Digital Pin 9 corresponds to ATmega328p's PB1 pin, which has Timer1 functionality
#define PULSE_DURATION 600  // Duration of a pulse in microseconds
#define SPACE_DURATION 600  // Duration of a space in microseconds

volatile bool transmitting = false;
volatile uint8_t currentByte = 0;
volatile uint8_t bitCount = 0;
volatile const char *message = NULL;
volatile bool pulseActive = false;
volatile unsigned long pulseEndTime = 0;

void setup() {
  Serial.begin(9600); // For verification purposes
  DDRB |= (1 << DDB1); // Set PB1 as output

  // Configure Timer1 for carrier frequency generation
  TCCR1A = 0;
  TCCR1B = 0;

  // Set desired frequency
  int frequency = 2125;
  int prescalar = 8;
  int timerValue = (F_CPU / (2 * prescalar * frequency)) - 1;

  // Set timer value
  OCR1A = timerValue;

  // Configure Timer1 in CTC mode (clears timer on compare)
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS11);

  // Enable Timer1 compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  // Configure Timer2 for pulse and space timing
  TCCR2A = 0;
  TCCR2B = 0;
  TCCR2B |= (1 << CS21); // Set prescaler to 8
  OCR2A = (F_CPU / 16 / 1000000) * PULSE_DURATION - 1; // Set Timer2 compare value for pulse duration

  // Set interrupt global enable flag bit (re-enable interrupts after being disabled).
  sei();
}

ISR(TIMER1_COMPA_vect) {
  PORTB ^= (1 << PORTB1); // Toggle PB1 to generate carrier frequency
}

ISR(TIMER2_COMPA_vect) {
  if (transmitting) {
    if (pulseActive) {
      // End of pulse
      PORTB &= ~(1 << PORTB1); // Set PB1 LOW
      pulseActive = false;
      OCR2A = (F_CPU / 16 / 1000000) * SPACE_DURATION - 1; // Set Timer2 compare value for space duration
    } else {
      // End of space
      if (bitCount < 8) {
        if (currentByte & (1 << (7 - bitCount))) {
          // Start pulse
          PORTB |= (1 << PORTB1); // Set PB1 HIGH
          pulseActive = true;
          OCR2A = (F_CPU / 16 / 1000000) * PULSE_DURATION - 1; // Set Timer2 compare value for pulse duration
        }
        bitCount++;
      } else {
        // Space between characters
        bitCount = 0;
        message++;
        if (*message) {
          currentByte = *message;
        } else {
          transmitting = false;
          TIMSK2 &= ~(1 << OCIE2A); // Disable Timer2 interrupt
        }
      }
    }
  }
}

void loop() {
  transmitMessage("Hello World!");
  while (transmitting) {
    // Wait for transmission to complete
  }
  Serial.println("Message transmitted successfully");
  //delay(5000);
}

void transmitMessage(const char* msg) {
  message = msg;
  currentByte = *message;
  bitCount = 0;
  transmitting = true;
  TIMSK2 |= (1 << OCIE2A); // Enable Timer2 interrupt
}
