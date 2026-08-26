/*
 * TriggerRuntime — the other half of TriggerSetup: no I2C at all.
 *
 * After TriggerSetup has run once, the expander watches its condition on its
 * own and drives a digital output pin. Your sketch reads that pin with plain
 * digitalRead(). There is no library call in this loop, no bus transaction,
 * and nothing to go wrong at a bad moment — the same as reading a limit switch.
 *
 * Wire expander digital output 0 to Arduino pin 2, and share a ground.
 */
const uint8_t TRIGGER_PIN = 2;

void setup() {
  Serial.begin(115200);
  pinMode(TRIGGER_PIN, INPUT);
}

void loop() {
  if (digitalRead(TRIGGER_PIN) == HIGH) {
    Serial.println(F("target in range"));
  }
  delay(50);
}
