 volatile uint32_t lastEdgeD2 = 0;
volatile uint32_t lastEdgeD3 = 0;
#define BAUDRATE      38400     // Serial output

volatile bool isChannelA0 = true;

#define ADC_THRESHOLD 86  // Equivalent to ~0.42V on 5V reference

// Registers & State
volatile uint32_t periodSpace = 0;   // Timer2 counts (low phase)
volatile uint32_t periodMark = 0;   // Timer2 counts (low phase)
uint32_t now = micros();

volatile uint8_t lastByte = 0;     // Received byte buffer
volatile uint32_t bitPos = 0;       // Current bit position (0-8)
float frequencySpace = 0;
float frequencyMark = 0;

volatile uint8_t writeAble = 0;
bool cycleAble = 0;
bool lastStateSpace = false;
bool lastStateMark = false;
bool isStartbit = false;
bool waitingData = true;
bool lastBit = false;
bool validData = false;

unsigned long bitTimer = 0;
const float bitLength = 14000; // Bit length in microseconds ≈ 22002 µs

//---------------------------------------------------------------------
// Setup: Configure comparators, timers, and serial
//---------------------------------------------------------------------
void setup() {
  Serial.begin(BAUDRATE);
  pinMode(2, INPUT);  // D2 = signal 1
  pinMode(3, INPUT);  // D3 = signal 2

  // Attach interrupts to both pins on rising edge
  attachInterrupt(digitalPinToInterrupt(2), isrSignal1, RISING);
  attachInterrupt(digitalPinToInterrupt(3), isrSignal2, RISING);
}

//---------------------------------------------------------------------
//Frequency read
//---------------------------------------------------------------------
void isrSignal1() {
  uint32_t now = micros();
  uint32_t period = now - lastEdgeD2;
  lastEdgeD2 = now;
  periodSpace = period;
}

void isrSignal2() {
  uint32_t now = micros();
  uint32_t period = now - lastEdgeD3;
  lastEdgeD3 = now;
  periodMark = period;
}

//---------------------------------------------------------------------
// Process Periods: Classify frequency and decode bits
//---------------------------------------------------------------------
void processPeriods(uint32_t periodSpace1, uint32_t periodMark1) {

  // Classify with ±4% tolerance (ADJUST AS NEEDED)
  /*
  bool isMark  = (frequencyMark > 2250 && frequencyMark < 2340);  // 2295Hz ±2%
  bool isSpaceF = (frequencySpace > 2080 && frequencySpace < 2170);  // 2125Hz ±2%
  */
  bool isMark = (periodMark > 427 && periodMark < 444);
  bool isSpaceF = (periodSpace > 461 && periodSpace < 481);
  // Ignore if neither or both (noise or invalid)
  if (!isMark && !isSpaceF) {
    validData = false;
  }
  else if (isMark && isSpaceF) {
    validData = false;
  }
  else if (isMark && lastBit){
    writeAble++;
    validData = true;
  }
  else if (isSpaceF && !lastBit){
    writeAble++;
    validData = true;
  }

  if (validData && isMark && writeAble == 0) {
    lastBit = 1;
  }
  else if (validData && isSpaceF && writeAble == 0) {
    lastBit = 0;
  }

  Serial.print(writeAble);
  Serial.print(lastBit);
  Serial.print(isSpaceF);
  Serial.print(bitPos);
  Serial.print(cycleAble);
  Serial.print(isStartbit);
  Serial.print(waitingData);
  Serial.print(" ");
  Serial.print(lastByte);
  Serial.print(" ");
  Serial.print(isMark);
  Serial.println(isSpaceF);

  // Protocol Decoding
  if (isSpaceF && waitingData && writeAble > 2) {       // Start bit (Space)
    writeAble = 3;
    isStartbit = 1;
    waitingData = 0;
    bitPos = 1;
  } 
  else if (bitPos > 0 && bitPos <= 8 && writeAble > 2) { // Data bits
    
    writeAble = 3;

    if (cycleAble){
      if (lastByte) {
      lastByte |= (1 << (bitPos - 1));
      }
      else if (!lastByte) {
      lastByte &= ~(1 << (bitPos - 1));
      }

      bitPos++;
      writeAble = 0;
    }
    
  } 
  else if (bitPos > 8 && isMark) {    // Stop bit (Mark)
    Serial.println((char)lastByte);
    waitingData = 1;
    lastByte = 0;
    bitPos = 0;
    writeAble = 0;
  }
  else if (bitPos > 8){
    
  }
}

//---------------------------------------------------------------------
// Main Loop: Debug prints and housekeeping
//---------------------------------------------------------------------
void loop() {
  if(writeAble > 3) writeAble = 3;
  if (isStartbit && (micros() - bitTimer > bitLength)) {
    // Start counting bit timerO
    bitTimer = micros();
    cycleAble = false;
    isStartbit = 0;
    writeAble = 0;
  }
  else if(micros() - bitTimer > bitLength && !waitingData){
    cycleAble = true;
    bitTimer = micros();
    writeAble =0;
  }
  else{
    cycleAble = false;
    isStartbit = false;
  }
  processPeriods(periodSpace, periodMark);
//  Serial.println(lastByte, BIN);

}
