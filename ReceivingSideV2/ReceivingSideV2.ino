// Pin and frequency definitions
#define IR_RECEIVER_PIN A0 // Analog pin for LTR-516AD input
#define MARK_FREQ 2295     // Frequency for logical 1 (mark)
#define SPACE_FREQ 2125    // Frequency for logical 0 (space)
#define BAUD_RATE 45.45    // Baud rate (bits per second)
#define BIT_DURATION (1000000 / BAUD_RATE) // Bit duration in microseconds (22 ms)

// Struct to encapsulate receiver state
struct ReceiverState {
  bool receiving;
  uint8_t currentByte;
  uint8_t bitCount;
  bool isStartBit;
  unsigned long lastPulseTime;
  bool newDataAvailable;
};

// Global instance of the receiver state
ReceiverState receiver;

// Function prototypes
void initializePins();
void initializeTimer1();
void initializeTimer2();
void handleIRSignal();
void processReceivedData(ReceiverState &state);

void setup() {
  Serial.begin(9600); // For verification purposes

  // Initialize receiver state
  receiver.receiving = false;
  receiver.currentByte = 0;
  receiver.bitCount = 0;
  receiver.isStartBit = true;
  receiver.lastPulseTime = 0;
  receiver.newDataAvailable = false;

  // Initialize pins, timers, and interrupts
  initializePins();
  initializeTimer1();
  initializeTimer2();

  // Attach interrupt for IR receiver input
  attachInterrupt(digitalPinToInterrupt(IR_RECEIVER_PIN), []() { handleIRSignal(receiver); }, CHANGE);

  // Set interrupt global enable flag bit (re-enable interrupts after being disabled).
  sei();
}

void loop() {
  // Check if new data is available
  if (receiver.newDataAvailable) {
    processReceivedData(receiver);
    receiver.newDataAvailable = false; // Reset the flag
  }
}

// Function to initialize pins
void initializePins() {
  pinMode(IR_RECEIVER_PIN, INPUT); // Set LTR-516AD pin as input
}

// Function to initialize Timer1 for frequency measurement
void initializeTimer1() {
  TCCR1A = 0; // Clear Timer1 control register A
  TCCR1B = 0; // Clear Timer1 control register B
  TCCR1B |= (1 << WGM12); // Set CTC mode
  TCCR1B |= (1 << CS11); // Set prescaler to 8
}

// Function to initialize Timer2 for bit timing
void initializeTimer2() {
  TCCR2A = 0; // Clear Timer2 control register A
  TCCR2B = 0; // Clear Timer2 control register B
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20); // Set prescaler to 1024
  OCR2A = (F_CPU / 1024 / (1000000 / BIT_DURATION)) - 1; // Set Timer2 compare value for bit duration
  TIMSK2 |= (1 << OCIE2A); // Enable Timer2 compare interrupt
}

// Function to handle IR signal detection
void handleIRSignal(ReceiverState &state) {
  unsigned long currentTime = micros();
  unsigned long pulseDuration = currentTime - state.lastPulseTime;

  if (pulseDuration > 1000) { // Ignore noise (adjust threshold as needed)
    if (pulseDuration >= (1000000 / SPACE_FREQ) * 0.9 && pulseDuration <= (1000000 / SPACE_FREQ) * 1.1) {
      // Detected space frequency (logical 0)
      if (state.isStartBit) {
        // Start bit detected
        state.isStartBit = false;
        state.receiving = true;
        state.currentByte = 0;
        state.bitCount = 0;
      } else if (state.receiving && state.bitCount < 8) {
        // Data bit (logical 0)
        state.currentByte &= ~(1 << state.bitCount); // Clear the bit
        state.bitCount++;
      }
    } else if (pulseDuration >= (1000000 / MARK_FREQ) * 0.9 && pulseDuration <= (1000000 / MARK_FREQ) * 1.1) {
      // Detected mark frequency (logical 1)
      if (state.receiving && state.bitCount < 8) {
        // Data bit (logical 1)
        state.currentByte |= (1 << state.bitCount); // Set the bit
        state.bitCount++;
      } else if (state.receiving && state.bitCount == 8) {
        // Stop bit detected
        state.receiving = false;
        state.newDataAvailable = true; // Set flag to indicate new data is available
      }
    }
  }

  state.lastPulseTime = currentTime;
}

// Function to process received data
void processReceivedData(ReceiverState &state) {
  Serial.print((char)state.currentByte); // Print the received character
}

// Timer2 compare interrupt service routine
ISR(TIMER2_COMPA_vect) {
  // This ISR is used for timing the bit duration
  if (receiver.receiving && receiver.bitCount < 8) {
    // Wait for the next bit
  } else if (receiver.receiving && receiver.bitCount == 8) {
    // Stop bit received, reset for the next byte
    receiver.receiving = false;
    receiver.isStartBit = true;
    receiver.newDataAvailable = true; // Set flag to indicate new data is available
  }
}
