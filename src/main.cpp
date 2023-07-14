#include <Arduino.h>
/****************************************************************************************************/
/*  1 MHz ADC conversions of 1 analog input (A0) triggered by Timer Counter 0 channel 2 TIOA2       */
/*  1 MHz DAC output on channel 1 (DAC1) triggered by Timer Counter 0 channel 2 TIOA2               */
/****************************************************************************************************/

#define DUE_CLOCK_RATE 84000000
#define TIMER_FREQUENCY 100000
#define ADC_FILTER 64 //filter out deviations of less than this from a 0-4095 signal

unsigned short mapDac = 0, adc = 0;
float voltsIn = 0.0, voltsDac = 0.0, readVolts = 0.0;
unsigned long currentMillis = 0, startMillis = 0, count = 0;

//// main compressor parameters. Adjust these to your needs. ////
int attack_f = 10;  // attack period (how soon the compressor will start attenuating loud signals) given in measurement frame
                    // units (see window_ms). Default setting corresponds to 50ms. Max buf_len / 2. Min 4.
int release_f = 40; // release period (how soon the compressor will soften attenuation after signals have become more silent),
                    // given in measurement frame units. Default setting corresponds to 200ms; Max buf_len.
                    // Does not have an effect if <= attack_f
int threshold = 18; // minimum signal amplitude before the compressor will kick in. Each unit corresponds to roughly 5mV
                    // peak-to-peak.
float ratio = 2.0;  // dampening applied to signals exceeding the threshold. n corresponds to limiting the signal to a level of
                    // threshold level plus 1/3 of the level in excess of the threshold (if possible: see duty_min, below)
                    // 1(min) = no attenuation; 20(max), essentially limit to threshold, aggressively
const float max_transition_rate = 1.11; // although the moving averages for attack and release will result in smooth transitions
                    // of the compression rate in most regular cases sudden signal spikes can result in abrupt transitions, introducing
                    // additional artefacts. This limits the maximum speed of the transition to +/- 11% of current value.

//// Some further constants that you will probably not have to tweak ////
#define DEBUG 1           // serial communication appears to introduce audible noise ("ticks"), thus debugging is disabled by default
const int window_ms = 5;  // milliseconds per measurement window. A narrow window will allow finer control over attack and release,
                          // but it will also cripple detection of low frequency amplitudes. Probably you don't want to change this.
const int buf_len = 100;  // size of buffer. attack_f and release_f cannot exceed this.
const int duty_min = 10;  // ceiling value for attenuation (lower values = more attenuation, 0 = off, 255 = no attenuation)
                          // beyond a certain value further attenuation is just too coarse grained for good results. Ideally, this
                          // value is never reached, but might be for aggressive dampening ratio and low thresholds.
const int duty_warn = 2 * duty_min;  // See above. At attenuation beyond this (i.e. smaller numbers), warning LED will flash.
                          // Reaching this point on occasion is quite benign. Reaching this point much of the time means too strong
                          // signal, too low threshold setting, or too aggressive inv_ratio.
const int signal_warn = 300;  // A warning LED will flash for signals exceeding this amplitude (5mv per unit, peak-to-peak) as
                          // it is probably (almost) too much for the circuit too handle (default value corresponds to about +-750mV
                          // in order to stay below the .8V typical 2N7000 body diode forward voltage, as well as below
                          // the 1.7V signal swing (centered at 3.3V) that the Arduino can handle).

//// Adjustable pin assignments
const int pin_led_warn = 13;
const int pin_led_high = 12;
const int pin_led_mid = 11;
const int pin_led_low = 10;

//// working variables ////
volatile int cmin = 4096; // minimum amplitude found in current measurement window
volatile int cmax = 0;    // maximum amplitude found in current measurement window
int buf[buf_len];         // ring buffer for moving averages / sums
int pos = 0;              // current buffer position
int attack_mova = 0;      // moving average (actually sum) of amplitudes over past attack period
int release_mova = 0;     // moving average (actually sum) of amplitudes over past release period
int32_t now = 0;          // start time of current loop
int32_t lastMillis = 0;         // time of lastMillis loop
int duty = 255;           // current PWM duty cycle for attenuator switch(es) (0: hard off, 255: no attenuation)
byte display_hold = 0;
float invratio = 1 / ratio;  // inverse of ratio. Saves some floating point divisions

#if DEBUG
int it = 0;
#endif

void adc_setup();
void dac_setup();
void tc_setup();
void ADC_Handler();
void indicateLevels(int rawval, int cduty);

void setup() {
  Serial.begin(115200);
  pinMode(pin_led_low, OUTPUT);
  pinMode(pin_led_mid, OUTPUT);
  pinMode(pin_led_high, OUTPUT);
  pinMode(pin_led_warn, OUTPUT);
  adc_setup();
  dac_setup();
  tc_setup();
  for (int i = 0; i < buf_len; ++i) {  // clear buffer
    buf[i] = 0;
  }
  startMillis = millis();
  lastMillis = millis();
}

void loop() {
  count++;
  currentMillis = millis();
  if ((currentMillis - startMillis) % 100 == 0 && count >1000) {
    Serial.print("count ");
    Serial.print(count);
    Serial.print("   adc offset: ");
    Serial.print(adc-2048);
    Serial.print("    cmax: ");
    Serial.print(cmax);
    Serial.print("    cmin: ");
    Serial.print(cmin);
    Serial.print("    duty: ");
    Serial.println(duty);
    count = 0;
    //vanilla analogread + analogwrite = 57,847 Hz
    //  1 MHz timer interrupt = 285,235 Hz
    // 48 kHz timer interrupt = 322,209 Hz 
    // 96 kHz timer interrupt = 315,365 Hz
    //384 kHz timer interrupt = 302,344 Hz
  }
  if (currentMillis < lastMillis || currentMillis - lastMillis > window_ms) {  // measurment window elapsed (or timer overflow)
    lastMillis = currentMillis;
  } else return;

  // get amplitude in current meausrement window, and set up next window
  if (++pos >= buf_len) pos = 0;
  int val = cmax - cmin;
  if (val < 0) val = 0;
  cmax = 0;
  cmin = 4096;

  // update the two moving averages (sums)
  int old_pos = pos - attack_f;
  if (old_pos < 0) old_pos += buf_len;
  attack_mova += val - buf[old_pos];
  old_pos = pos - release_f;
  if (old_pos < 0) old_pos += buf_len;
  release_mova += val - buf[old_pos];

  // store new value in ring buffer
  buf[pos] = val;

  // calculate new attenuation settings
  // first caculate based on attack period
  const int attack_threshold = threshold * attack_f;
  int attack_duty = 255;
  if (attack_mova > attack_threshold) {
    const int target_level = attack_threshold * pow ((float) attack_mova / attack_threshold, invratio);
    // Instead of the logrithmic volume calculation above, the faster linear one below seems too yield
    // acceptable results, too. Hoever, the Arduino is fast enough, so we do the "real" thing.
    //   const int target_level = (attack_mova - attack_threshold) / ratio + attack_threshold;
    attack_duty = (255 * (int32_t) target_level) / attack_mova;
    #if DEBUG
      if ((currentMillis - startMillis) % 100 == 0 && count >1000) {
        Serial.print(attack_mova);
        Serial.print("-");
        Serial.print(attack_threshold);
        Serial.print("-");
        Serial.print(ratio);
        Serial.print("-");
        Serial.print(target_level);
        Serial.print("-");
        Serial.println(attack_duty);
      }
    #endif
  }
  // if the new duty setting is _below_ the current, based on attack period, check release window to see, if
  // the time has come to release attenuation, yet:
  if (attack_duty < duty) duty = max (attack_duty, duty / max_transition_rate);
  else {
    int release_duty = 255;
    const int release_threshold = threshold * release_f;
    if (release_mova > release_threshold) {
      const int target_level = release_threshold * pow ((float) release_mova / release_threshold, invratio);
      release_duty = (255 * (int32_t) target_level) / release_mova;
    } else {
      release_duty = 255;
    }
    if (release_duty >= duty) duty = min (release_duty, duty * max_transition_rate);
    #if DEBUG
      else {
        //Serial.println("hold");
      }
    #endif
  }
  
  indicateLevels(val, duty);
}

void indicateLevels (int rawval, int cduty) {
  digitalWrite (pin_led_warn, rawval >= signal_warn);
  digitalWrite (pin_led_high, cduty <= duty_warn);
  digitalWrite (pin_led_mid, cduty < 128);
  digitalWrite (pin_led_low, cduty != 255);
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
  if (adc < cmin) cmin = adc;
  if (adc > cmax) cmax = adc;
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