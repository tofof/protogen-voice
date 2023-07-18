#include <Arduino.h>
#include <Yin.h>

#define DUE_CLOCK_RATE 84000000             // due processor speed, 84 MHz
#define TIMER_FREQUENCY YIN_SAMPLING_RATE   // audio sampling (ADC conversion) speed; clock speed will be half this
#define PLAYBACK_BUFFER_SIZE 32768          // must be power of 2
#define PLAYBACK_SPEED 131072                  // adjustment sensitivity, i.e. >100 for <=1% shifts
#define ADC_FILTER 64                       // filter out deviations of less than this from a 0-4095 signal
#define SIN_STEEP 3.0                       // steepness (1.0 to ~32.0) of sin function used to increase SNR, see https://www.desmos.com/calculator/wdtfsassev
#define log2(x) (log(x) * M_LOG2E)          // Arduino doesn't have a log2 function :(


short adc = 0;
unsigned long currentMillis = 0, startMillis = 0, count = 0, conversions = 0;
unsigned short steepSinTable[4096];
short playbackData[PLAYBACK_BUFFER_SIZE], rawData[YIN_BUFFER_SIZE];
bool clipping = false;
volatile long playbackSpeed = PLAYBACK_SPEED;      // repitching speed, PLAYBACK_SPEED 1:1 playback, higher is faster
Yin yInMethod;
static float frequency;
static char* noteName = new char[4];

#define DEBUG 1

void buildSteepSinTable();
void adc_setup();
void dac_setup();
void tc_setup();
void ADC_Handler();
float getNearestNoteFrequency(float frequency);

void setup() {
  noInterrupts();
  Serial.begin(115200);
  buildSteepSinTable();
  adc_setup();
  dac_setup();
  tc_setup();

  // Prepare for frequency analysis
  Yin_init(&yInMethod, 0.45);

  Serial.println("Setup complete.");
  startMillis = millis();
  
  interrupts();
}

void loop() {
  while(!(ADC->ADC_ISR & ADC_ISR_EOC7)); // lock clock speed to half timer frequency
  count++;
  currentMillis = millis();
  if (conversions >= YIN_BUFFER_SIZE) {
    //#ifdef DEBUG
    static unsigned int t;
    t = millis();
    //#endif
    frequency = Yin_getPitch(&yInMethod, rawData);
    conversions = 0;
    #ifdef DEBUG
    Serial.print("Frequency: ");
    Serial.print(frequency);
    #endif
    
    if (frequency >= 20 && frequency < 3600) { // Was a pitch actually detected?
      static float newFreq = frequency;
      newFreq = (newFreq + frequency) / 2.0f;
      frequency = newFreq;

      // Work out what note to tune to
      float targetFrequency = getNearestNoteFrequency(frequency);

      // Using this, we then adjust the playbackSpeed value
      playbackSpeed = constrain(round(targetFrequency * PLAYBACK_SPEED / frequency), 1, (PLAYBACK_SPEED*8)-1); //was +ourSpeed
    }
    //#ifdef DEBUG
    Serial.print("    Playback speed: ");
    float pbs = 100.0*playbackSpeed/PLAYBACK_SPEED;
    Serial.print(pbs);
    Serial.print("%    Took ");
    Serial.print(millis() - t);
    Serial.print("ms");
    Serial.println("");
    //#endif
  }

  // if (clipping) {
  //     Serial.println(" ***************** CLIPPING ***************** ");
  //     clipping = false;
  //   }
}

// noteName can be NULL or a char* with space for 3 characvters + null terminator
float getNearestNoteFrequency(float frequency) {
  // Define the note names
  static const char* const NoteNames[] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};


  // Calculate the number of semitones from C2 (used as a reference point)
  float nearestSemitoneFromC2f = 12.0 * log2(frequency / 65.41);
  uint16_t nearestSemitoneFromC2 = round(nearestSemitoneFromC2f);

  // How I had the above line set when I was recording my Cher/Believe Karaoke Version
  // uint16_t nearestSemitoneFromC2 = round(nearestSemitoneFromC2f/2)*2;

  // Calculate the detected note 
  if (noteName) {
    uint8_t noteNumber = nearestSemitoneFromC2 % 12;
    strcpy(noteName, NoteNames[noteNumber]);
    // Add the Octave
    uint8_t octave = 2 + (nearestSemitoneFromC2 / 12);
    char tmp[4];
    itoa(octave, tmp, 10);
    strcat(noteName, tmp);

    #ifdef DEBUG
    Serial.print("    Nearest Note: ");
    Serial.print(noteName);
    Serial.print(" (");
    Serial.print(pow(2, nearestSemitoneFromC2 / 12.0f) * 65.41f);
    Serial.print(" Hz)");
    #endif 
  }

  // noteDif gives us -0.5 to 0.5 of how in-tune we are.
  float noteDiff = nearestSemitoneFromC2 - nearestSemitoneFromC2f;

  return pow(2, nearestSemitoneFromC2 / 12.0f) * 65.41f;
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
  static unsigned short inputPosition = 0;  
  static unsigned long outputPosition = 0;
  
  adc = ADC->ADC_CDR[7];                    // Reading ADC->ADC_CDR[i] clears EOCi bit
  //if (conversions < YIN_BUFFER_SIZE) {
    rawData[conversions] = adc-2048;
    conversions++;
  //}

  adc = steepSinTable[adc];
  if (abs(adc-2048) < ADC_FILTER) adc = 2048;
  if (abs(adc-2048) == 2048) clipping = true;
  playbackData[inputPosition] = adc;
  inputPosition = (inputPosition+1) & (PLAYBACK_BUFFER_SIZE-1); 
  outputPosition += playbackSpeed; //playbackSpeed 128 will be >> 7 so advances 1, ie 1:1 playback

  //Play from a different part
  DACC->DACC_CDR = playbackData[outputPosition >> 17];

}

/*************  Configure dacc_setup function  *******************/
void dac_setup () {
  PMC->PMC_PCER1 = PMC_PCER1_PID38;                   // DACC power ON
  DACC->DACC_CR = DACC_CR_SWRST ;                     // Reset DACC
  DACC->DACC_MR = DACC_MR_TRGEN_EN                    // Hardware trigger select
                  | DACC_MR_TRGSEL(0b011)             // Trigger by TIOA2
                  | DACC_MR_USER_SEL_CHANNEL1        // select channel 0 or 1
                  | DACC_MR_REFRESH (1)
                  | DACC_MR_STARTUP_8
                  | DACC_MR_MAXS;
  DACC->DACC_CHER = DACC_CHER_CH1;                   // enable channel 0 = DAC0  channel 1=DAC1
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

