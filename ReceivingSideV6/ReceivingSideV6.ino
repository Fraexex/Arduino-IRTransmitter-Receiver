// Pins
#define MARK   A0  // Comparator +input (via external filter)
#define SPACE  A1  // Threshold control (DAC)

// Settings
#define BAUDRATE      9600     // Serial output
#define SAMPLE_RATE   100      // Debug print interval (ms)
#define TIMEOUT_MS    20       // Reset if no edges for this long
#define HYSTERESIS    10       // Comparator threshold hysteresis (in DAC steps)

// Registers & State
volatile uint16_t highPeriod = 0;  // Timer1 counts (high phase)
volatile uint16_t lowPeriod = 0;   // Timer2 counts (low phase)
volatile uint8_t lastByte = 0;     // Received byte buffer
volatile uint8_t bitPos = 0;       // Current bit position (0-8)
volatile uint32_t lastEdgeTime = 0; // Timeout tracker

// Moving average filter (window=4)
#define FILTER_SIZE  4
uint32_t periodBuffer[FILTER_SIZE] = {0};
uint8_t bufferIndex = 0;

//---------------------------------------------------------------------
// Setup: Configure comparators, timers, and serial
//---------------------------------------------------------------------
void setup() {
  Serial.begin(BAUDRATE);

  // Configure Analog Comparator
  ADCSRB = (1 << ACME);       // Enable comparator multiplexer
  ADMUX = (1 << REFS0);       // Use AVcc as reference
  ACSR = (1 << ACIE) | (1 << ACIS1) | (1 << ACIS0); // Interrupt on both edges

  // Configure Timers (62.5ns/count @ 16MHz, no prescaler)
  TCCR1A = 0;                 // Timer1 (16-bit, high phase)
  TCCR1B = (1 << CS10);       
  TCCR2A = 0;                 // Timer2 (8-bit, low phase)
  TCCR2B = (1 << CS20);       

  // Initialize threshold (adjust dynamically later)
  analogWrite(SPACE, 23);      // Start with ~0.45V threshold
}

//---------------------------------------------------------------------
// Analog Comparator ISR: Captures rising/falling edges
//---------------------------------------------------------------------
ISR(ANALOG_COMP_vect) {
  static bool lastState = false;
  lastEdgeTime = micros();     // Reset timeout

  if (ACSR & (1 << ACO)) {    // Rising edge (signal > threshold)
    if (!lastState) {
      lowPeriod = TCNT2;      // Capture Timer2 (low phase)
      TCNT2 = 0;              // Reset Timer2
      TCCR1B = (1 << CS10);   // Start Timer1
      lastState = true;
    }
  } 
  else {                      // Falling edge (signal < threshold)
    if (lastState) {
      highPeriod = TCNT1;     // Capture Timer1 (high phase)
      TCNT1 = 0;              // Reset Timer1
      TCCR2B = (1 << CS20);   // Start Timer2
      lastState = false;

      // Apply moving average filter to reduce noise
      uint32_t totalPeriod = (highPeriod + lowPeriod) * 0.0625; // µs
      periodBuffer[bufferIndex] = totalPeriod;
      bufferIndex = (bufferIndex + 1) % FILTER_SIZE;
      
      // Process filtered period
      processPeriods(filteredPeriod());
    }
  }
}

//---------------------------------------------------------------------
// Moving Average Filter: Smoothes period measurements
//---------------------------------------------------------------------
uint32_t filteredPeriod() {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < FILTER_SIZE; i++) {
    sum += periodBuffer[i];
  }
  return sum / FILTER_SIZE;
}

//---------------------------------------------------------------------
// Process Periods: Classify frequency and decode bits
//---------------------------------------------------------------------
void processPeriods(uint32_t period) {
  // Calculate frequency (Hz)
  float frequency = 1e6 / (float)period;

  // Classify with ±4% tolerance (ADJUST AS NEEDED)
  bool isMark  = (frequency > 2200 && frequency < 2390);  // 2295Hz ±4%
  bool isSpaceF = (frequency > 2040 && frequency < 2210);  // 2125Hz ±4%

  // Ignore if neither or both (noise or invalid)
  if (!isMark && !isSpaceF) return;
  if (isMark && isSpaceF) return;

  // Protocol Decoding
  if (isSpaceF && bitPos == 0) {       // Start bit (Space)
    lastByte = 0;
    bitPos = 1;
  } 
  else if (bitPos > 0 && bitPos <= 8) { // Data bits
    if (isMark) lastByte |= (1 << (8 - bitPos));
    bitPos++;
  } 
  else if (bitPos > 8 && isMark) {    // Stop bit (Mark)
    Serial.print("Received: 0x");
    Serial.println(lastByte, HEX);
    bitPos = 0;
  }
}

//---------------------------------------------------------------------
// Timeout Check: Resets if no edges detected for TIMEOUT_MS
//---------------------------------------------------------------------
void checkTimeout() {
  if ((micros() - lastEdgeTime) > TIMEOUT_MS && bitPos != 0) {
    bitPos = 0;  // Reset if stuck mid-byte
    Serial.println("Timeout: Resetting decoder");
  }
}

//---------------------------------------------------------------------
// Dynamic Threshold Adjustment (optional)
//---------------------------------------------------------------------
void adjustThreshold() {
  static uint32_t lastAdjust = 0;
  if (micros() - lastAdjust > 1000) {  // Adjust every 1s
    int markVal = analogRead(MARK);
    int spaceVal = analogRead(SPACE);
    int newThreshold = (markVal + spaceVal) / 4;  // Empirical scaling
    analogWrite(SPACE, newThreshold);
    lastAdjust = micros();
  }
}

//---------------------------------------------------------------------
// Main Loop: Debug prints and housekeeping
//---------------------------------------------------------------------
void loop() {
  static uint32_t lastPrint = 0;
  
  checkTimeout();       // Handle sync loss
  adjustThreshold();    // Optional dynamic threshold

  // Debug prints (rate-limited)
  if (micros() - lastPrint > SAMPLE_RATE) {
    Serial.print("Period: ");
    Serial.print(filteredPeriod());
    Serial.println(" µs");
    lastPrint = micros();
  }
}
