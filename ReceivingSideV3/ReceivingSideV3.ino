#define IR_PIN 2  // Digital pin (INT0) connected to diode OR output

// Timing thresholds (adjust based on actual measurements)
#define MARK_MIN_US   400   // ~436µs expected
#define MARK_MAX_US   450
#define SPACE_MIN_US  451
#define SPACE_MAX_US  500   // ~471µs expected

volatile uint8_t rxByte = 0;
volatile uint8_t bitCount = 0;
volatile bool receiving = false;
volatile unsigned long lastEdge = 0;

void setup() {
  Serial.begin(9600);
  pinMode(IR_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(IR_PIN), handleInterrupt, CHANGE);
}

void loop() {
  // Main logic here (e.g., process received bytes)
}

void handleInterrupt() {
  unsigned long now = micros(); // Current time in microseconds (resets after ~70 minutes)
  unsigned long pulseWidth = now - lastEdge; // Duration between two consecutive edges (high→low or low→high)
  Serial.println(pulseWidth);
  lastEdge = now; // Timestamp (in microseconds) of the previous signal change (rising or falling edge)

  // Noise filter (ignore glitches <100µs)
  if (pulseWidth < 100) return;

  // Classify pulse
  bool isMark = (pulseWidth >= MARK_MIN_US && pulseWidth <= MARK_MAX_US);
  bool isSpace = (pulseWidth >= SPACE_MIN_US && pulseWidth <= SPACE_MAX_US);

  // Protocol state machine
  if (!receiving) {
    if (isSpace) {  // Start bit detected
      receiving = true;
      rxByte = 0;
      bitCount = 0;
    }
  } else {
    if (bitCount < 8) {  // Data bits
      if (isMark) rxByte |= (1 << bitCount);
      bitCount++;
    } else {  // Stop bit
      if (isMark) Serial.write(rxByte);  // Valid byte received
      receiving = false;
    }
  }
}
