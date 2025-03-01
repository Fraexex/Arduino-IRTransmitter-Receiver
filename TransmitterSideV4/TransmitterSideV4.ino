#define IR_LED_PIN 9 // Digital Pin 9 corresponds to ATmega328p's PB1 pin
#define MARK_FREQ 2295 // Frequency for logical 1 (mark)
#define SPACE_FREQ 2125 // Frequency for logical 0 (space)
#define BAUD_RATE 45.45 // Baud rate (bits per second)
#define BIT_DURATION (1000000 / BAUD_RATE) // Bit duration in microseconds (22 ms)

volatile bool transmitting = false;
volatile uint8_t currentByte = 0;
volatile uint8_t bitCount = 0;
volatile const char *message = NULL;
volatile bool isStartBit = true;

void setup() {
  Serial.begin(9600); // For verification purposes
  DDRB |= (1 << DDB1); // Set PB1 as output

  // Configure Timer1 for mark/space frequency generation (consider just making this as a function)
  TCCR1A = 0; // Clear Timer1 control register A
  TCCR1B = 0; // Clear Timer1 control register B
  TCCR1B |= (1 << WGM12); // Set CTC mode
  TCCR1B |= (1 << CS11); // Set prescaler to 8

  // Configure Timer2 for bit timing
  TCCR2A = 0; // Clear Timer2 control register A
  TCCR2B = 0; // Clear Timer2 control register B
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20); // Set prescaler to 1024
  OCR2A = (F_CPU / 1024 / (1000000 / BIT_DURATION)) - 1; // Set Timer2 compare value for bit duration

  // Enable Timer1 and Timer2 compare interrupts
  TIMSK1 |= (1 << OCIE1A);
  TIMSK2 |= (1 << OCIE2A);

  // Set interrupt global enable flag bit (re-enable interrupts after being disabled).
  sei();
}

ISR(TIMER1_COMPA_vect) {
  PORTB ^= (1 << PORTB1); // Toggle PB1 to generate mark/space frequency
}

ISR(TIMER2_COMPA_vect) {
  if (transmitting) {
    if (isStartBit) {
      // Transmit start bit (logical 0)
      OCR1A = (F_CPU / (2 * 8 * SPACE_FREQ)) - 1; // Set Timer1 for space frequency
      isStartBit = false;
    } else if (bitCount < 8) {
      // Transmit data bits (LSB first)
      if (currentByte & (1 << bitCount)) {
        OCR1A = (F_CPU / (2 * 8 * MARK_FREQ)) - 1; // Set Timer1 for mark frequency
        Serial.print("1");
      } else {
        OCR1A = (F_CPU / (2 * 8 * SPACE_FREQ)) - 1; // Set Timer1 for space frequency
        Serial.print("0");
      }
      bitCount++;
    } else {
      Serial.print(" ");
      // Transmit stop bit (logical 1)
      OCR1A = (F_CPU / (2 * 8 * MARK_FREQ)) - 1; // Set Timer1 for mark frequency
      bitCount = 0;
      message++;
      if (*message) {
        currentByte = *message;
        isStartBit = true;
      } else {
        transmitting = false;
        TIMSK2 &= ~(1 << OCIE2A); // Disable Timer2 interrupt
      }
    }
  }
}

void loop() {
  transmitMessage("Hello World!");
  while (transmitting) {
    // Wait for transmission to complete
  }
  Serial.println();
  Serial.println("Message transmitted successfully");
  delay(5000);
}

void transmitMessage(const char* msg) {
  message = msg;
  currentByte = *message;
  bitCount = 0;
  isStartBit = true;
  transmitting = true;
  TIMSK2 |= (1 << OCIE2A); // Enable Timer2 interrupt
}
