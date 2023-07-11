#include <Arduino.h>

#define AUDIO_INPUT     A0
#define margin 100
#define base 2136
const uint32_t sinsize = 256;
uint32_t sinus[sinsize];
unsigned short mapDac = 0, adc = 0;
float voltsIn = 0.0, voltsDac = 0.0, readVolts = 0.0;
unsigned long currentMillis = 0, startMillis = 0, count = 0;



// put function declarations here:
//int myFunction(int, int);

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  analogWriteResolution(12);
  pinMode(AUDIO_INPUT, INPUT);
  pinMode(DAC0, OUTPUT);
  for (uint32_t i = 0; i < sinsize; i++)   // Fill Sinus buffer
  {
    uint32_t chsel = (0 << 12) | (1 << 28);               // MSB [31:16]on DAC channel 1, LSB [15:0] on DAC channel 0
    sinus[i]  = 2047 * sin(i * 2 * PI / sinsize) + 2047; //  0 < sinus [i] < 4096
    sinus[i] |= sinus[i] << 16 | chsel;
  }

  //dacc_setup();
  startMillis = millis();
}

int16_t incoming = base;
int16_t average = base;

void loop() {
  // incoming = analogRead(AUDIO_INPUT) - base;
  // average = (average + average + incoming) / 3.0;
  // if (abs(incoming) > margin) {
  //   Serial.println(incoming);
  // }
  // // else {
  // //   Serial.print("Average: ");
  // //   Serial.println(average);
  // // }

  // https://forum.arduino.cc/t/project-with-the-dac-on-the-arduino-due/270803
  adc = count % 4096;
  voltsIn = adc * (3.3 / 4095);
  mapDac = map(adc, 682, 3413, 0, 4095);  // 1/6 => 682, 5/6 => 3413
  if (adc<683) mapDac = 0;
  if (adc>3413) mapDac = 4095;
  readVolts = analogRead(AUDIO_INPUT) * (3.3 / 4095);
  analogWrite(DAC0, mapDac);

  Serial.print("adc ");
  Serial.print(adc);
  Serial.print("  voltsIn ");
  Serial.print(voltsIn, 4);
  Serial.print("  mapDac ");
  Serial.print(mapDac);
  Serial.print("  readVolts ");
  Serial.print(readVolts, 4);
  Serial.print("  difference ");
  Serial.println(voltsIn - readVolts, 4);

  count++;
  currentMillis = millis();
  // if ((currentMillis - startMillis) % 1000 == 0 && count >1000) {
  //   Serial.print("count ");
  //   Serial.println(count);
  //   count = 0;
  // }
  delay(1);
}

// https://forum.arduino.cc/t/due-dac-update-rate-resolution/450943
void dacc_setup () {
  PIOB->PIO_PDR |= PIO_PDR_P15 | PIO_PDR_P16;  // Disable GPIO on corresponding pins DAC0 and DAC1
  PMC->PMC_PCER1 |= PMC_PCER1_PID38 ;     // DACC power ON
  DACC->DACC_CR = DACC_CR_SWRST ;         // reset DACC
  DACC->DACC_MR = DACC_MR_REFRESH (1)
                  | DACC_MR_STARTUP_0
                  | DACC_MR_MAXS
                  | DACC_MR_TAG_EN
                  | DACC_MR_WORD_WORD;
  DACC->DACC_CHER = DACC_CHER_CH0 | DACC_CHER_CH1;      // enable DAC channel 0 and DAC channel 1
}

void dac_write () {
  static uint32_t Index = 0;
  while (!(DACC->DACC_ISR && DACC_ISR_TXRDY));  // Wait until DACC is ready for a new conversion
  DACC->DACC_CDR = (uint32_t) sinus[Index];  // Request a new conversion
  Index++;
  if (Index == sinsize) Index = 0;
}