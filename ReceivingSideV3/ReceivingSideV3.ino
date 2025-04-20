// Pins
#define IR_PIN 2  // Digital pin (INT0) connected to LM311's diode OR output
//#define ANALOG_COMP_PIN 6  // AIN0 (analog comparator positive input)
#define REF_PIN 7  // AIN1 (analog comparator negative input) 0.4V reference
#define DEBUG_LED 13 // Built-in LED to indicate activity

// Timing thresholds (adjust based on actual measurements)
#define MARK_MIN_US   400   // ~436µs expected
#define MARK_MAX_US   450   // ~436µs expected
#define SPACE_MIN_US  451   // ~471µs expected
#define SPACE_MAX_US  500   // ~471µs expected
#define NOISE_THRESHOLD 100   // Ignore pulses shorter than this
#define BIT_TIMEOUT     25000 // 25ms timeout for incomplete bytes (µs)

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
  volatile unsigned long lastEdge;
  volatile unsigned long lastValidEdge;     // Timestamp for pulse width
  volatile uint16_t debugPulseCount;
  
} ReceiverState;

volatile ReceiverState receiver;
unsigned long lastDebugPrint = 0;

void setup() {
  memset((void*)&receiver, 0, sizeof(receiver));
  
  configAnalogComparator();
  pinMode(IR_PIN, INPUT);
  pinMode(DEBUG_LED, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(IR_PIN), handleInterrupt, CHANGE);
  digitalWrite(IR_PIN, LOW);

  Serial.begin(9600);
  Serial.println("IR Receiver Ready:");
  analogWrite(REF_PIN, 10);
}

void loop() {
  processReceivedData();
  handleDebugOutput();
  debug();
}

// ============ CORE FUNCTIONS ============

void handleInterrupt() {
  const unsigned long now = micros(); // Duration between two consecutive edges (high→low or low→high). Start counting when it's above 2.3V and below 2.3V. Add together to get period.
  // Serial.print("Pulse width (µs): ");
  // Serial.println(pulseWidth);
  if (now - receiver.lastValidEdge < NOISE_THRESHOLD) return; // Ignore edges too close together

  const unsigned long pulseWidth = now - receiver.lastEdge;
  receiver.lastEdge = now;
  // receiver.lastEdge = now; // Timestamp (in microseconds) of the previous signal change (rising or falling edge)

  // Noise filter (ignore glitches <1µs)
  if (pulseWidth < NOISE_THRESHOLD) return;

  // Classify pulse
  const bool isMarkFreq = inRange(pulseWidth, MARK_MIN_US, MARK_MAX_US);
  const bool isSpaceFreq = inRange(pulseWidth, SPACE_MIN_US, SPACE_MAX_US);

  // debug();
  
  // Protocol state machine
  if (!receiver.receiving && isSpaceFreq) {  // Start bit
    startReception();
    receiver.lastValidEdge = now;
  } 
  else if (receiver.receiving) {
    if (processBit(isMarkFreq, now)) {
      receiver.lastValidEdge = now;
    }
  }

  // Debug counting
  if (isMarkFreq || isSpaceFreq) {
    receiver.lastValidEdge = now;
    receiver.debugPulseCount++;
  }
}

bool processBit(bool isMark, unsigned long timestamp) {
  static unsigned long bitStartTime = 0;
  
  // Initialize timer for new byte
  if (receiver.bitCount == 0) {
    bitStartTime = timestamp;
  }
  
  // Timeout check
  if (timestamp - bitStartTime > BIT_TIMEOUT) {
    receiver.receiving = false;
    return false;
  }
  
  // Store bit
  if (receiver.bitCount < 8) {
    if (isMark) {
      receiver.rxByte |= (1 << (7 - receiver.bitCount)); // LSB→MSB
    }
    receiver.bitCount++;
    return true;
  }
  // Stop bit
  else if (isMark) {
    storeByte();
    receiver.receiving = false;
    return true;
  }
  
  return false;
}

void configAnalogComparator() {
  // Configure analog comparator for Schmitt trigger mode (p. 202)
  ACSR = 0; // Clear the register
  ACSR |= (1 << ACIS1) | (1 << ACIS0); // Set interrupt on both edges
  ACSR |= (1 << ACBG);  // Use internal bandgap reference (1.1V)

  DIDR1 |= (1 << AIN0D) | (1 << AIN1D); // Disable digital input buffers
}

void processReceivedData() {
  while (dataAvailable()) {
    char c = readByte();
    Serial.print(c);
    digitalWrite(DEBUG_LED, !digitalRead(DEBUG_LED));
  }
}

uint8_t readByte() {
  uint8_t currentByte = receiver.buffer[receiver.tail];
  receiver.tail = (receiver.tail + 1) % BUF_SIZE;
  receiver.bufferFull = false;
  return currentByte;
}

int readIRPin() {
  return (ACSR & (1 << ACO)) ? HIGH : LOW;
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

bool dataAvailable() {
  return (receiver.head != receiver.tail) || receiver.bufferFull;
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

void handleDebugOutput() {
  static unsigned long lastDebugTime = 0;
  if (millis() - lastDebugTime > 1000) {
    Serial.print("\n[Status] Buffer: ");
    Serial.print((receiver.head - receiver.tail + BUF_SIZE) % BUF_SIZE);
    Serial.println("/32 used");
    lastDebugTime = millis();
  }
}

void printVoltage() {
  int raw = analogRead(IR_PIN);        // Read ADC (0-1023)
  float voltage = raw * (5.0 / 1023.0);       // Convert to volts (5V Arduino)
  Serial.print("Voltage: ");
  Serial.print(voltage, 2);                   // Print with 2 decimal places
  Serial.println("V");
}

void debug() {
  Serial.print("rxByte: ");
  Serial.println(receiver.rxByte);
  Serial.print("Bit count: ");
  Serial.println(receiver.bitCount);
  Serial.print("Receiving bool: ");
  Serial.println(receiver.receiving);
  Serial.print("Last pulse edge: ");
  Serial.println(receiver.lastValidEdge);
  printVoltage();
}
