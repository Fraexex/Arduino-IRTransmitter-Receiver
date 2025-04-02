#define IR_PIN 2  // Digital pin (INT0) connected to LM311's diode OR output

// Timing thresholds (adjust based on actual measurements)
#define MARK_MIN_US   400   // ~436µs expected
#define MARK_MAX_US   450
#define SPACE_MIN_US  451
#define SPACE_MAX_US  500   // ~471µs expected

volatile uint8_t rxByte = 0;             // Stores received byte
volatile uint8_t bitCount = 0;           // Tracks bit position
volatile bool receiving = false;         // Reception in progress
volatile unsigned long lastEdge = 0;     // Timestampt for pulse width 
volatile bool newDataReady = false;      // Flag for completed byte

void setup() {
  Serial.begin(9600);
  pinMode(IR_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(IR_PIN), handleInterrupt, CHANGE);
}

void loop() {
  // Process received bytes)
  if (newDataReady) {
    Serial.print("New data: ");
    Serial.println((char)rxByte);
    newDataReady = false;
  }
}

void handleInterrupt() {
  unsigned long now = micros(); // Current time in microseconds (resets after ~70 minutes)
  unsigned long pulseWidth = now - lastEdge; // Duration between two consecutive edges (high→low or low→high)
  Serial.print("Pulse width (µs): ");
  Serial.println(pulseWidth);
  lastEdge = now; // Timestamp (in microseconds) of the previous signal change (rising or falling edge)

  // Noise filter (ignore glitches <100µs)
  if (pulseWidth < 100) return;

  // Classify pulse
  bool isMark = (pulseWidth >= MARK_MIN_US && pulseWidth <= MARK_MAX_US);
  bool isSpace = (pulseWidth >= SPACE_MIN_US && pulseWidth <= SPACE_MAX_US);

  debug();
  
  // Protocol state machine
  if (!receiving && isSpace) {  // Start bit
    receiving = true;
    rxByte = 0;
    bitCount = 0;
    debug();
  } 
  else if (receiving) {
    if (bitCount < 8) {  // Data bits
      if (isMark) rxByte |= (1 << (7 - bitCount));
      debug();
      bitCount++;
    }
    else if (isMark) {  // Stop bit
      Serial.write(rxByte);  // Valid byte received
      newDataReady = true;
      receiving = false;
      debug();
    }
  }
}

void debug() {
  Serial.print("rxByte: ");
  Serial.println(rxByte);
  Serial.print("Bit count: ");
  Serial.println(bitCount);
  Serial.print("Receiving bool: ");
  Serial.println(receiving);
  Serial.print("Last pulse edge: ");
  Serial.println(lastEdge);
  Serial.print("New data bool: ");
  Serial.println(newDataAvailable);
}
