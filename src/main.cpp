#include <Arduino.h>
/****************************************************************************************************/
/*  1 MHz ADC conversions of 1 analog input (A0) triggered by Timer Counter 0 channel 2 TIOA2       */
/*  1 MHz DAC output on channel 1 (DAC1) triggered by Timer Counter 0 channel 2 TIOA2               */
/****************************************************************************************************/

#define DUE_CLOCK_RATE 84000000
#define TIMER_FREQUENCY 100000
#define ADC_FILTER 48 //filter out deviations of less than this from a 0-4095 signal

unsigned short mapDac = 0, adc = 0;
float voltsIn = 0.0, voltsDac = 0.0, readVolts = 0.0;
unsigned long currentMillis = 0, startMillis = 0, count = 0;

void adc_setup();
void dac_setup();
void tc_setup();
void ADC_Handler();

void setup() {
  Serial.begin(115200);
  adc_setup();
  dac_setup();
  tc_setup();
  startMillis = millis();
}

void loop() {
  count++;
  currentMillis = millis();
  if ((currentMillis - startMillis) % 100 == 0 && count >1000) {
    Serial.print("count ");
    Serial.print(count);
    Serial.print("   adc offset: ");
    Serial.println(adc-2048);
    count = 0;
    //vanilla analogread + analogwrite = 57,847 Hz
    //  1 MHz timer interrupt = 285,235 Hz
    // 48 kHz timer interrupt = 322,209 Hz 
    // 96 kHz timer interrupt = 315,365 Hz
    //384 kHz timer interrupt = 302,344 Hz
  }
  // delay(1);
}

/*************  Configure adc_setup function  *******************/
void adc_setup() {
  PMC->PMC_PCER1 |= PMC_PCER1_PID37;                    // ADC power ON
  ADC->ADC_CR = ADC_CR_SWRST;                           // Reset ADC
  ADC->ADC_MR |=  ADC_MR_TRGEN_EN                       // Hardware trigger select
                  | ADC_MR_TRGSEL_ADC_TRIG3             // Trigger by TIOA2
                  | ADC_MR_PRESCAL(1);
  ADC->ADC_ACR = ADC_ACR_IBCTL(0b01);                   // For frequencies > 500 KHz
  ADC->ADC_IER = ADC_IER_EOC7;                          // End Of Conversion interrupt enable for channel 7
  NVIC_EnableIRQ(ADC_IRQn);                                  // Enable ADC interrupt
  ADC->ADC_CHER = ADC_CHER_CH7;                         // Enable Channel 7 = A0
}

void ADC_Handler() {
  /* Todo : Apply any digital filtering before DAC output  */
  /* Beware : Stay in ADC_Handler much less than 1 us  !!! */
  adc = ADC->ADC_CDR[7];                    // Reading ADC->ADC_CDR[i] clears EOCi bit
  if (abs(adc-2048) < ADC_FILTER) adc = 2048;
  DACC->DACC_CDR = adc;
}

/*************  Configure dacc_setup function  *******************/
void dac_setup () {
  PMC->PMC_PCER1 = PMC_PCER1_PID38;                   // DACC power ON
  DACC->DACC_CR = DACC_CR_SWRST ;                     // Reset DACC
  DACC->DACC_MR = DACC_MR_TRGEN_EN                    // Hardware trigger select
                  | DACC_MR_TRGSEL(0b011)             // Trigger by TIOA2
                  | DACC_MR_USER_SEL_CHANNEL0         // select channel 0
                  | DACC_MR_REFRESH (1)
                  | DACC_MR_STARTUP_8
                  | DACC_MR_MAXS;
  DACC->DACC_CHER = DACC_CHER_CH0;                   // enable channel 0 = DAC0
}

/*************  Timer Counter 0 Channel 2 to generate PWM pulses thru TIOA2  ************/
void tc_setup() {
  PMC->PMC_PCER0 |= PMC_PCER0_PID29;                      // TC2 power ON : Timer Counter 0 channel 2 IS TC2
  TC0->TC_CHANNEL[2].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1  // MCK/2, clk on rising edge
                              | TC_CMR_WAVE               // Waveform mode
                              | TC_CMR_WAVSEL_UP_RC       // UP mode with automatic trigger on RC Compare
                              | TC_CMR_ACPA_CLEAR         // Clear TIOA2 on RA compare match
                              | TC_CMR_ACPC_SET;          // Set TIOA2 on RC compare match
  TC0->TC_CHANNEL[2].TC_RC = DUE_CLOCK_RATE/2 / TIMER_FREQUENCY;  //TC_RC=(84MHz/2)/Frequency so TC_RC = 875 for Frequency 48 kHz
  TC0->TC_CHANNEL[2].TC_RA = DUE_CLOCK_RATE/2 / TIMER_FREQUENCY / 2;  //Any Duty cycle in between 1 and TC_RC, 50% by default
  TC0->TC_CHANNEL[2].TC_CCR = TC_CCR_SWTRG | TC_CCR_CLKEN;// Software trigger TC2 counter and enable
}