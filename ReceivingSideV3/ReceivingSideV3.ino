#define IR_PIN 2  // Digital pin (INT0) connected to LM311's diode OR output

// Timing thresholds (adjust based on actual measurements)
#define MARK_MIN_US   400   // ~436µs expected
#define MARK_MAX_US   450   // ~436µs expected
#define SPACE_MIN_US  451   // ~471µs expected
#define SPACE_MAX_US  500   // ~471µs expected
#define NOISE_THRESHOLD 100 // Ignore pulses shorter than this

// Buffer to prevent data loss, due to slow loop (adjust based on data rate)
#define BUF_SIZE 32

// Receiver State
typedef struct {
  // Buffer variables
  volatile uint8_t buffer[BUF_SIZE];
  volatile uint8_t head;
  volatile uint8_t tail;
  volatile bool bufferFull;
  
  // Interrupt service routine variables
  volatile uint8_t rxByte;             // Stores received byte
  volatile uint8_t bitCount;           // Tracks bit position
  volatile bool receiving;             // Reception in progress
  volatile unsigned long lastEdge;     // Timestamp for pulse width 
} ReceiverState;

volatile ReceiverState receiver;

void setup() {
  receiver.head = 0;
  receiver.tail = 0;
  receiver.bufferFull = false;
  receiver.rxByte = 0;
  receiver.bitCount = 0;
  receiver.receiving = false;
  receiver.lastEdge = 0;
  
  Serial.begin(9600);
  pinMode(IR_PIN, INPUT);
  digitalWrite(IR_PIN, LOW);
  attachInterrupt(digitalPinToInterrupt(IR_PIN), handleInterrupt, CHANGE);
}

void loop() {
  processReceivedData();
  int state = digitalRead(IR_PIN);
  Serial.println(state);
  // debug();
}

void processReceivedData() {
  while (dataAvailable()) {
    Serial.print((char)readByte());
  }
}

bool dataAvailable() {
  return (receiver.head != receiver.tail) || receiver.bufferFull;
}

uint8_t readByte() {
  uint8_t currentByte = receiver.buffer[receiver.tail];
  receiver.tail = (receiver.tail + 1) % BUF_SIZE;
  receiver.bufferFull = false;
  return currentByte;
}

void handleInterrupt() {
  const unsigned long now = micros(); // Current time in microseconds (resets after ~70 minutes)
  const unsigned long pulseWidth = now - receiver.lastEdge; // Duration between two consecutive edges (high→low or low→high)
  // Serial.print("Pulse width (µs): ");
  // Serial.println(pulseWidth);
  receiver.lastEdge = now; // Timestamp (in microseconds) of the previous signal change (rising or falling edge)

  // Noise filter (ignore glitches <100µs)
  if (pulseWidth < 100) return;

  // Classify pulse
  const bool isMarkFreq = inRange(pulseWidth, MARK_MIN_US, MARK_MAX_US);
  const bool isSpaceFreq = inRange(pulseWidth, SPACE_MIN_US, SPACE_MAX_US);

  debug();
  
  // Protocol state machine
  if (!receiver.receiving && isSpaceFreq) {  // Start bit
    startReception();
  } 
  else if (receiver.receiving) {
    processBit(isMarkFreq);
  }
}

// ============ HELPER FUNCTIONS ============

bool inRange(unsigned long value, unsigned long minimum, unsigned long maximum) {
  return (value >= minimum && value <= maximum);
}

void startReception() {
  receiver.receiving = true;
  receiver.rxByte = 0;
  receiver.bitCount = 0;
}

void processBit(bool isMarkFreq) {
  if (receiver.bitCount < 8) {
    if (isMarkFreq) {
      receiver.rxByte |= (1 << (7 - receiver.bitCount)); // LSB->MSB
    }
    receiver.bitCount++;
  }
  else if (isMarkFreq) {
    storeByte();
    receiver.receiving = false;
  }
}

void storeByte() {
  if (!receiver.bufferFull) {
    receiver.buffer[receiver.head] = receiver.rxByte;
    receiver.head = (receiver.head + 1) % BUF_SIZE;
    receiver.bufferFull = (receiver.head == receiver.tail);
  }
}

void debug() {
  Serial.print("rxByte: ");
  Serial.println(receiver.rxByte);
  Serial.print("Bit count: ");
  Serial.println(receiver.bitCount);
  Serial.print("Receiving bool: ");
  Serial.println(receiver.receiving);
  Serial.print("Last pulse edge: ");
  Serial.println(receiver.lastEdge);
}
