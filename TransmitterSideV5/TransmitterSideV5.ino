/*
 * Hybrid IR RTTY Transmitter
 * - Uses Timer1 for precise frequency generation (mark/space)
 * - Uses Timer2 for accurate bit timing
 * - Implements circular buffer for message queuing
 * - Direct AVR register manipulation for efficiency
 * - Current limiting for IR LED (0.4mA max)
 */

// Hardware Configuration
#define IR_LED_PIN    9       // Digital Pin 9 corresponds to ATmega328p's PB1 pin (OC1A)
#define DEBUG_LED     13      // Onboard LED for visual confirmation

// Protocol Parameters
#define MARK_FREQ     2295    // Frequency for logical 1 (mark) in Hz
#define SPACE_FREQ    2125    // Frequency for logical 0 (space) in Hz
#define BAUD_RATE     45.45   // Standard RTTY baud rate
#define BIT_DURATION  22000   // (1000000 / BAUD_RATE) in μs

// Calculated Timer Values
#define MARK_OCR1A    435     //((F_CPU / (2 * 8 * MARK_FREQ)) - 1)  // ~435 for 16MHz
#define SPACE_OCR1A   470     //((F_CPU / (2 * 8 * SPACE_FREQ)) - 1) // ~470 for 16MHz
#define BIT_OCR2A     217     // For ~22ms with 1024 prescaler at 16MHz

// Buffer Configuration
#define TX_BUFFER_SIZE 64     // Transmit buffer size

// Transmitter State
volatile struct {
  // Message buffer (circular)
  char buffer[TX_BUFFER_SIZE];
  uint8_t head, tail;
  bool bufferFull;
  
  // Transmission state
  volatile bool transmitting;
  volatile uint8_t currentByte;
  volatile uint8_t bitCount;
  volatile bool isStartBit;
  volatile bool isStopBit;
} txState;

// Initialize Timer1 for mark/space frequency generation
void setupTimer1() {
  // Reset Timer1 control registers
  TCCR1A = 0;
  TCCR1B = 0;
  
  // Configure Timer1 for CTC mode (Clear Timer on Compare Match)
  TCCR1B |= (1 << WGM12);
  
  // Set prescaler to 8
  TCCR1B |= (1 << CS11);
  
  // Set initial frequency to MARK
  OCR1A = MARK_OCR1A;
  
  // Enable Timer1 compare match interrupt
  TIMSK1 |= (1 << OCIE1A);
}

// Initialize Timer2 for bit timing
void setupTimer2() {
  // Reset Timer2 control registers
  TCCR2A = 0;
  TCCR2B = 0;
  
  // Configure Timer2 for CTC mode
  TCCR2A |= (1 << WGM21);
  
  // Set prescaler to 1024
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);
  
  // Set compare value for ~22ms bit duration
  OCR2A = BIT_OCR2A;
  
  // Timer2 interrupt is enabled only when transmitting
}

// Set frequency to MARK (logical 1)
void setMarkFrequency() {
  OCR1A = MARK_OCR1A;
}

// Set frequency to SPACE (logical 0)
void setSpaceFrequency() {
  OCR1A = SPACE_OCR1A;
}

// Timer1 interrupt service routine - toggles IR LED pin to generate carrier frequency
ISR(TIMER1_COMPA_vect) {
  PORTB ^= (1 << PORTB1); // Toggle PB1 (digital pin 9) to generate square wave
}

// Timer2 interrupt service routine - handles bit timing for RTTY transmission
ISR(TIMER2_COMPA_vect) {
  // If not transmitting, do nothing
  if (!txState.transmitting) {
    // Disable Timer2 interrupt
    TIMSK2 &= ~(1 << OCIE2A);
    return;
  }
  
  // Handle start bit
  if (txState.isStartBit) {
    setSpaceFrequency();
    txState.isStartBit = false;
    PORTB |= (1 << PORTB5); // Turn on debug LED (digital pin 13)
    return;
  }
  
  // Handle stop bit
  if (txState.isStopBit) {
    setMarkFrequency();
    txState.isStopBit = false;
    
    // Check if there's more data to transmit
    if (txState.head != txState.tail) {
      // Get next byte from buffer
      txState.currentByte = txState.buffer[txState.tail];
      txState.tail = (txState.tail + 1) % TX_BUFFER_SIZE;
      txState.bufferFull = false;
      
      // Start new byte transmission
      txState.bitCount = 0;
      txState.isStartBit = true;
    } else {
      // End transmission
      txState.transmitting = false;
      PORTB &= ~(1 << PORTB5); // Turn off debug LED
      TIMSK2 &= ~(1 << OCIE2A); // Disable Timer2 interrupt
    }
    return;
  }
  
  // Handle data bits
  if (txState.bitCount < 8) {
    // Transmit data bits (LSB first)
    if (txState.currentByte & (1 << txState.bitCount)) {
      setMarkFrequency();
    } else {
      setSpaceFrequency();
    }
    txState.bitCount++;
    
    // If we've sent all data bits, prepare for stop bit
    if (txState.bitCount >= 8) {
      txState.isStopBit = true;
    }
  }
}

// Add a byte to the transmit buffer
bool queueByte(uint8_t byte) {
  // Check if buffer is full
  if (txState.bufferFull) {
    return false;
  }
  
  // Store byte in buffer
  txState.buffer[txState.head] = byte;
  txState.head = (txState.head + 1) % TX_BUFFER_SIZE;
  
  // Check if buffer is now full
  txState.bufferFull = (txState.head == txState.tail);
  
  // Start transmission if not already running
  if (!txState.transmitting) {
    startTransmission();
  }
  
  return true;
}

// Start transmitting data from the buffer
void startTransmission() {
  // Check if there's data to transmit
  if (txState.head == txState.tail) {
    return;
  }
  
  // Disable interrupts while setting up transmission
  cli();
  
  // Get first byte from buffer
  txState.currentByte = txState.buffer[txState.tail];
  txState.tail = (txState.tail + 1) % TX_BUFFER_SIZE;
  txState.bufferFull = false;
  
  // Initialize transmission state
  txState.bitCount = 0;
  txState.isStartBit = true;
  txState.isStopBit = false;
  txState.transmitting = true;
  
  // Enable Timer2 interrupt for bit timing
  TIMSK2 |= (1 << OCIE2A);
  
  // Re-enable interrupts
  sei();
}

// Queue a string for transmission
void transmitString(const char* str) {
  // Queue each character
  while (*str) {
    // Wait if buffer is full
    while (!queueByte(*str)) {
      // Allow interrupts to process queued data
      delay(10);
    }
    str++;
  }
}

void setup() {
  Serial.begin(115200);
  
  // Configure IR LED pin as output
  DDRB |= (1 << DDB1);  // Set PB1 (digital pin 9) as output
  DDRB |= (1 << DDB5);  // Set PB5 (digital pin 13) as output
  
  // Initialize transmitter state
  memset((void*)&txState, 0, sizeof(txState));
  
  // Setup timers
  setupTimer1();
  setupTimer2();
  
  // Enable global interrupts
  sei();
  
  // Send startup message to serial
  Serial.println("IR RTTY Transmitter Ready");
  
  // Test transmission
  transmitString("Hello World!");
}

void loop() {
  // Check for new data from serial
  if (Serial.available() > 0) {
    char c = Serial.read();
    queueByte(c);
    
    // Echo to serial
    Serial.write(c);
  }
}
