#include <Arduino.h>

#define AUDIO_INPUT     A0

// put function declarations here:
int myFunction(int, int);

void setup() {
  pinMode(AUDIO_INPUT, INPUT);
}

void loop() {
  Serial.println(analogRead(AUDIO_INPUT));
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}