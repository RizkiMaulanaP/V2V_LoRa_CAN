// ==========================================
// PinCycle — cycle outputs HIGH one at a time
// ==========================================
// Lights each pin HIGH for 3 s in turn (the others are LOW),
// then moves on to the next and wraps around.
//
//   GP4  -> Buzzer
//   GP17 -> LED1
//   GP6  -> LED2
//   GP5  -> LED3

#include "Arduino.h"

// Pins in the order they should be cycled
const uint8_t PINS[] = {4, 17, 6, 5};
const uint8_t NUM_PINS = sizeof(PINS) / sizeof(PINS[0]);

#define CYCLE_INTERVAL 3000   // ms each pin stays HIGH

uint8_t  currentPin = 0;
uint32_t lastSwitch = 0;

void allLow() {
  for (uint8_t i = 0; i < NUM_PINS; i++) {
    digitalWrite(PINS[i], LOW);
  }
}

void setup() {
  Serial.begin(115200);

  for (uint8_t i = 0; i < NUM_PINS; i++) {
    pinMode(PINS[i], OUTPUT);
    digitalWrite(PINS[i], LOW);
  }

  // Start on the first pin
  currentPin = 0;
  digitalWrite(PINS[currentPin], HIGH);
  lastSwitch = millis();
  Serial.printf("HIGH -> GP%u\n", PINS[currentPin]);
}

void loop() {
  if (millis() - lastSwitch >= CYCLE_INTERVAL) {
    allLow();
    currentPin = (currentPin + 1) % NUM_PINS;
    digitalWrite(PINS[currentPin], HIGH);
    lastSwitch = millis();
    Serial.printf("HIGH -> GP%u\n", PINS[currentPin]);
  }
}
