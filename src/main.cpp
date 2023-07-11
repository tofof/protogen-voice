#include <Arduino.h>

#define AUDIO_INPUT     A0
unsigned short mapDac = 0, adc = 0;
float voltsIn = 0.0, voltsDac = 0.0, readVolts = 0.0;
unsigned long currentMillis = 0, startMillis = 0, count = 0;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  analogWriteResolution(12);
  pinMode(AUDIO_INPUT, INPUT);
  pinMode(DAC0, OUTPUT);
  startMillis = millis();
}

void loop() {
  // https://forum.arduino.cc/t/project-with-the-dac-on-the-arduino-due/270803
  adc = count % 4096;
  voltsIn = adc * (3.3 / 4095);
  mapDac = map(adc, 682, 3413, 0, 4095);  // 1/6 => 682, 5/6 => 3413
  if (adc<683) mapDac = 0;
  if (adc>3413) mapDac = 4095;
  readVolts = analogRead(AUDIO_INPUT) * (3.3 / 4095);
  analogWrite(DAC0, mapDac);

  // Serial.print("adc ");
  // Serial.print(adc);
  // Serial.print("  voltsIn ");
  // Serial.print(voltsIn, 4);
  // Serial.print("  mapDac ");
  // Serial.print(mapDac);
  // Serial.print("  readVolts ");
  // Serial.print(readVolts, 4);
  // Serial.print("  difference ");
  // Serial.println(voltsIn - readVolts, 4);

  count++;
  currentMillis = millis();
  if ((currentMillis - startMillis) % 1000 == 0 && count >1000) {
    Serial.print("count ");
    Serial.println(count);
    count = 0;
  }
  // delay(1);
}