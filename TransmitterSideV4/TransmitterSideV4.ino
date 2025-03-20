/*
 * Put states in an object, since they change frequency.
 * Only would need to look at one place to see how states
 * are being mutated.
 * 
 * One object that deals with sending messages
 */

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

  // Configure Timer1 for mark/space frequency generation
  timer1MarkSpace();

  // Configure Timer2 for bit timing
  timer2BitTiming();

  // Enable Timer1 and Timer2 compare interrupts 
  enableTimer1();
  enableTimer2();

  // Set interrupt global enable flag bit (re-enable interrupts after being disabled)
  sei();
}

ISR(TIMER1_COMPA_vect) {
  PORTB ^= (1 << PORTB1); // Toggle PB1 to generate mark/space frequency
}

ISR(TIMER2_COMPA_vect) {
  if (!transmitting) {
    return;
  }
  
  if (isStartBit) {
    // Transmit start bit (logical 0)
    setTimer1SpaceFreq();
    isStartBit = false;
  }else if (bitCount < 8) {
    // Transmit data bits (LSB first)
    if (currentByte & (1 << bitCount)) {
      setTimer1MarkFreq();
      //Serial.print("1");
    } else {
      setTimer1SpaceFreq();
      //Serial.print("0");
    }
    bitCount++;
  } else {
    //Serial.print(" ");
    // Transmit stop bit (logical 1)
    setTimer1MarkFreq();
    bitCount = 0;
    message++;
    if (*message) {
      currentByte = *message;
      isStartBit = true;
    } else {
      endMessage();
    }
  }
}

void loop() {
  transmitMessage("Hello World!");
  while (transmitting) {
    // Wait for transmission to complete
  }
  checkStates();
  delay(1000);
}

void timer1MarkSpace() {
  TCCR1A = 0; // Clear Timer1 control register A
  TCCR1B = 0; // Clear Timer1 control register B
  TCCR1B |= (1 << WGM12); // Set CTC mode
  TCCR1B |= ~(1 << CS10) | (1 << CS11) | ~(1 << CS12); // Set prescaler to 8
}

void setTimer1MarkFreq() {
  OCR1A = (unsigned long long)(F_CPU / (2 * 8 * MARK_FREQ)) - 1;
}

void setTimer1SpaceFreq() {
  OCR1A = (unsigned long long)(F_CPU / (2 * 8 * SPACE_FREQ)) - 1;
}

void timer2BitTiming() {
  TCCR2A = 0; // Clear Timer2 control register A
  TCCR2B = 0; // Clear Timer2 control register B
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20); // Set prescaler to 1024
  OCR2A = (unsigned long long)(F_CPU / 1024 / (1000000 / BIT_DURATION)) - 1; // Set Timer2 compare value for bit duration
}

void enableTimer1() {
  TIMSK1 |= (1 << OCIE1A);
}

void disableTimer1() {
  TIMSK1 &= ~(1 << OCIE1A);
}

void enableTimer2() {
  TIMSK2 |= (1 << OCIE2A); // Enable Timer2 interrupt (consider making OCIE2A a parameter)
}

void disableTimer2() {
  TIMSK2 &= ~(1 << OCIE2A); // Disable Timer2 interrupt
}

void transmitMessage(const char* msg) {
  message = msg;
  currentByte = *message;
  bitCount = 0;
  isStartBit = true;
  transmitting = true;
  enableTimer2();
}

void endMessage() {
  transmitting = false;
  disableTimer2();
}

void checkStates() {
  Serial.println("Message transmitted successfully");
  Serial.print("F_CPU: ");
  Serial.println(F_CPU);
  Serial.print("OCR1A: ");
  Serial.println(OCR1A);
  Serial.print("OCR2A: ");
  Serial.println(OCR2A);
  Serial.print("TIMSK1: ");
  Serial.println(TIMSK1);
  Serial.print("TIMSK2: ");
  Serial.println(TIMSK2);
  Serial.print("TIMSK1: ");
  Serial.println(TIMSK2);
  Serial.println();
}
