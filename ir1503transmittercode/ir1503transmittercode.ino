#include <IRremote.h>
#include <IRremoteInt.h>

// Using the IRremote library: https://github.com/Arduino-IRremote/Arduino-IRremote

IRsend irsend;

void setup() {
  Serial.begin(9600);
  pinMode(5, INPUT_PULLUP); // SW1 connected to pin 2
  pinMode(6, INPUT_PULLUP); // SW2 connected to pin 3
  pinMode(7, INPUT_PULLUP); // SW3 connected to pin 4
  // The IR LED is connected to pin 3 (PWM ~) on the Arduino
}

void loop() {
  
  if (digitalRead(5) == LOW) { // When SW1 is pressed
    irsend.sendNEC(0x34895725, 32);  // Replace with your own unique code
    Serial.println("Code sent!");
    delay(30);
  } 

  else if (digitalRead(6) == LOW) { // When SW2 is pressed
    irsend.sendNEC(0x56874159, 32); // Replace with your own unique code
    Serial.println("Code sent!");
    delay(30);
  } 

  else if (digitalRead(7) == LOW) { // When SW3 is pressed
    irsend.sendNEC(0x15467823, 32); // Replace with your own unique code
    Serial.println("Code sent!");
    delay(30);
  } 
  
  else {
    Serial.println("Nothing to send");
    delay(30);
  } 

  delay(100);
}
