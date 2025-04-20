/*
 * 
 */

#define IR_LED_PIN 9 // Digital Pin 9 corresponds to ATmega328p's PB1 pin, which has Timer1 functionality
#define PULSE_DURATION 600
#define SPACE_DURATION 600

volatile bool transmitting = false;
volatile uint8_t currentByte = 0;
volatile uint8_t bitCount = 0;
volatile const char *message = NULL;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // Verification purposes
  DDRB |= (1 << DDB1); // Set PB1 as output

  // Config Timer1
  TCCR1A = 0;
  TCCR1B = 0;

  // Set desired frequency
  int frequency = 2125;
  int prescalar = 8;
  int timerValue = (F_CPU / (2 * prescalar * frequency)) - 1;

  // Set timer value
  OCR1A = timerValue;

  // Config Timer1 in CTC mode (clears timer on compare)
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS11);

  // Enable Timer1 compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  // Config Timer2
  

  // Set interrupt global enable flag bit (re-enable interrupts after being disabled).
  sei();
}

ISR(TIMER1_COMPA_vect) {
  PORTB ^= (1 << PORTB1);
}

void loop() {
  // put your main code here, to run repeatedly:
  transmitMessage("Hello World!");
  while (transmitting) {
    // Wait for transmission to complete
  }
  Serial.println(1); // Serial monitor to print 1, if message successfully transmitted
  delay(5000);
}

// Implementation of sendPulse & sendSpace
void transmitMessage(const char* msg) {
  message = msg;
  currentByte = *message;
  bitCount = 0;
  transmitting = true;
  // Enable Timer2 interrupt
  while (*message) {  
    char c = *message; // Get current char
    for (int i = 0; i < 8; i++) {
      if (c & (1 << (7 - i))) { // Check if bit is 1
        sendPulse();
      } else {
        sendSpace();
      }
    }
    sendSpace(); // Space between chars
    message++; // Move to next char
  }
}

// What to do if HIGH voltage 
void sendPulse() {
  // Turn on IR LED
  PORTB |= (1 << PORTB1); // Set PB1 HIGH
  delayMicroseconds(PULSE_DURATION);

  PORTB &= ~(1 << PORTB1); // Set PB1 LOW
  delayMicroseconds(PULSE_DURATION);
}

void sendSpace() {
  PORTB &= ~(1 << PORTB1); // Set PB1 LOW
  delayMicroseconds(SPACE_DURATION);
}
