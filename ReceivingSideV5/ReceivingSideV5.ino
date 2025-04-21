/*RECEIVER.ino*/

// Hardware Configuration
#define IR_IN_PIN     A0      // Photodiode amplifier output
#define COMP_OUT_PIN  2       // Comparator output (INT0)
#define DEBUG_LED     13      // Onboard LED for visual confirmation

// Protocol Parameters
#define MARK_PERIOD   436     // 1/2295Hz in μs
#define SPACE_PERIOD  471     // 1/2125Hz in μs
#define BIT_PERIOD    22000   // (1/45.45)*1000000 μs
#define BIT_TIMEOUT   BIT_PERIOD  // *10 for char Timeout for incomplete reception
#define NOISE_THRESH  100     // μs

// Receiver State Machine
volatile struct {
  uint8_t buffer[64];
  uint8_t head, tail;
  bool bufferFull;
  
  uint8_t rxByte;
  uint8_t bitCount;
  bool receiving;
  uint32_t lastEdge;
  uint32_t bitStartTime;
} rxState;

// Setup analog comparator for frequency detection
void setupAnalogFrontEnd() {
  // Configure analog comparator
  ADCSRB = (1 << ACME);        // Connect ADC multiplexer to negative input
  ADMUX = (1 << REFS0) | 0;    // AREF = AVcc, select ADC0
  ACSR = (1 << ACIE) |         // Enable interrupt
         (1 << ACIS1) | (1 << ACIS0); // Trigger on rising edge
         
  pinMode(COMP_OUT_PIN, INPUT); // Optional - digital pin connected to comparator output
}

// Safe time difference calculation that handles uint32_t overflow
uint32_t timeDiff(uint32_t current, uint32_t previous) {
  // Handle overflow case
  return (current >= previous) ? (current - previous) : (0xFFFFFFFF - previous + current + 1);
}

// Analog comparator interrupt service routine
ISR(ANALOG_COMP_vect) {
  uint32_t now = micros();
  uint32_t pulseWidth = timeDiff(now, rxState.lastEdge);
  rxState.lastEdge = now;
  
  // Filter out noise
  if (pulseWidth < NOISE_THRESH) return;
  
  // Classify frequency based on period
  bool isMark = (abs((int)(pulseWidth - MARK_PERIOD)) < abs((int)(pulseWidth - SPACE_PERIOD)));
  
  // RTTY state machine
  if (!rxState.receiving && !isMark) {
    // Start bit detected (space/0)
    rxState.receiving = true;
    rxState.rxByte = 0;
    rxState.bitCount = 0;
    rxState.bitStartTime = now;
    
    digitalWrite(DEBUG_LED, HIGH);
  } 
  else if (rxState.receiving) {
    // Check if we're in a new bit period from the start time
    uint32_t bitOffset = timeDiff(now, rxState.bitStartTime);
    uint8_t bitPosition = bitOffset / BIT_PERIOD;
    
    // Sample in the middle of each bit
    if (bitPosition > rxState.bitCount && bitPosition <= 9) {
      // Process the previous bit
      if (bitPosition >= 2 && bitPosition <= 9) {
        // Data bits (skip start bit)
        if (isMark) {
          rxState.rxByte |= (1 << (bitPosition - 2));
        }
      }
      
      rxState.bitCount = bitPosition;
      
      // End of frame
      if (bitPosition == 9) {
        // Verify stop bit is mark
        if (isMark) {
          // Store complete byte if buffer not full
          if (!rxState.bufferFull) {
            rxState.buffer[rxState.head] = rxState.rxByte;
            rxState.head = (rxState.head + 1) % 64;
            rxState.bufferFull = (rxState.head == rxState.tail);
          }
        }
        
        // Reset for next byte
        rxState.receiving = false;
        digitalWrite(DEBUG_LED, LOW);
      }
    }
    
    // Timeout if we haven't received a complete byte
    if (timeDiff(now, rxState.bitStartTime) > 220000) {
      rxState.receiving = false;
      digitalWrite(DEBUG_LED, LOW);
    }
  }
}

// Check if data is available in receive buffer
bool dataAvailable() {
  return rxState.head != rxState.tail;
}

// Read a byte from the receive buffer
uint8_t readByte() {
  if (rxState.head == rxState.tail) {
    return 0; // Buffer empty
  }
  
  uint8_t data = rxState.buffer[rxState.tail];
  rxState.tail = (rxState.tail + 1) % 64;
  rxState.bufferFull = false;
  
  return data;
}

void setup() {
  Serial.begin(115200);
  pinMode(DEBUG_LED, OUTPUT);
  
  // Initialize receiver state
  memset((void*)&rxState, 0, sizeof(rxState));
  
  // Setup analog front-end for IR detection
  setupAnalogFrontEnd();
  
  // Enable analog comparator interrupt
  ACSR |= (1 << ACIE);
  
  Serial.println("IR Receiver Ready");
}

void loop() {
  // Check if we have received data
  if (dataAvailable()) {
    uint8_t data = readByte();
    Serial.write(data);
    
    // Toggle LED to show data received
    digitalWrite(DEBUG_LED, !digitalRead(DEBUG_LED));
  }
}
