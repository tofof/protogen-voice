#include <Arduino.h>
#include <Yin.h>

#define DUE_CLOCK_RATE 84000000
#define TIMER_FREQUENCY 200000 // audio sampling (ADC conversion) speed; clock speed will be half this
#define ADC_FILTER 64 // filter out deviations of less than this from a 0-4095 signal
#define SIN_STEEP 3.0 // steepness (1.0 to ~32.0) of sin function used to increase SNR, see https://www.desmos.com/calculator/wdtfsassev
#define log2(x) (log(x) * M_LOG2E) // Arduino doesn't have a log2 function :(

unsigned short mapDac = 0, adc = 0;
float voltsIn = 0.0, voltsDac = 0.0, readVolts = 0.0;
unsigned long currentMillis = 0, startMillis = 0, count = 0, conversions = 0;
static uint16_t steepSinTable[4096];
bool clipping = false;
volatile byte speed = 128;

Yin yInMethod;

#define DEBUG 1


void buildSteepSinTable();
void adc_setup();
void dac_setup();
void tc_setup();
void ADC_Handler();

void setup() {
  noInterrupts();
  Serial.begin(115200);
  buildSteepSinTable();
  adc_setup();
  dac_setup();
  tc_setup();

  // Prepare for frequency analysis
  Yin_init(&yInMethod, 512, 0.05f);

  Serial.println("Setup complete.");
  startMillis = millis();
  interrupts();
}

void loop() {
  //while(!(ADC->ADC_ISR & ADC_ISR_EOC7)); // lock clock speed to half timer frequency
  count++;
  currentMillis = millis();
  if (clipping) {
      Serial.println(" ***************** CLIPPING ***************** ");
      clipping = false;
    }
  if ((currentMillis - startMillis) % 1000 == 0 && count >1000) {
    #if DEBUG
    Serial.print("Processing: ");
    Serial.print(count);
    Serial.print(" Hz");
    Serial.print("     Sampling: ");
    Serial.print(conversions);
    Serial.print(" Hz");
    // Serial.print("   adc0: ");
    // Serial.print(adc-2048);
    // Serial.print("   sin0: ");
    // Serial.print(steepSinTable[adc]-2048);
    Serial.println("");
    #endif
    count = 0;
    conversions = 0;
    //vanilla analogread + analogwrite = 57,847 Hz
    //  1 MHz timer interrupt = 285,235 Hz
    // 48 kHz timer interrupt = 322,209 Hz 
    // 96 kHz timer interrupt = 315,365 Hz
    //384 kHz timer interrupt = 302,344 Hz
  }
}

// steeper sin equation https://www.desmos.com/calculator/wdtfsassev
void buildSteepSinTable() {
  float x;            // position in table between 0 and 1
  float s;            // sinus value
  float k=SIN_STEEP;  // steepness between 0 and infinity (in practice keep k<32, as above approaches square wave; k>5250 is perfect square wave for 4096 points)
  for (int i=0; i<4096; i++) {
    x = i/4096.0;
    s = pow(0.5+sin(x*PI-PI/2)/2,pow(2*(1-x),k));
    steepSinTable[i] = 4096*s;
    // Serial.print(i);
    // Serial.print("   ");
    // Serial.println(steepSinTable[i]);
  }
  
  
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
  NVIC_EnableIRQ(ADC_IRQn);                             // Enable ADC interrupt
  ADC->ADC_CHER = ADC_CHER_CH7;                         // Enable Channel 7 = A0
}

void ADC_Handler() {
  /* Beware : Stay in ADC_Handler as little time as possible */
  adc = ADC->ADC_CDR[7];                    // Reading ADC->ADC_CDR[i] clears EOCi bit
  //if (abs(adc-2048) < ADC_FILTER) adc = 2048;
  if (abs(steepSinTable[adc] - 2048) == 2048) clipping = true;

  static unsigned short data[512];
  static unsigned short inputPosition = 0;  
  static unsigned short outputPosition = 0;
  data[inputPosition] = steepSinTable[adc];   // Store current sample
  inputPosition = (inputPosition+1) & 511;
  outputPosition += speed; //speed 128 will be >> 7 so advances 1, ie 1:1 playback
  
  // Play from a different part
  DACC->DACC_CDR = data[outputPosition >> 7]; //short 65535 >> 7 = 511, so in-bounds on array

  conversions++;
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