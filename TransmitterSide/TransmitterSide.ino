int irLedPin = 3;

void setup() {
  // put your setup code here, to run once:
  pinMode(irLedPin, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  // Generate square wave w/ f = 2125Hz
  tone(irLedPin, 2125);
  // delay(1000);

  // noTone(irLedPin);
}
