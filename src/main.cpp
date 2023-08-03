#include <Arduino.h>
#include <Yin.h>

#define log2(x) (log(x) * M_LOG2E)          // Arduino doesn't have a log2 function :(
#define DUE_CLOCK_RATE 84000000             // due processor speed, 84 MHz
#define TIMER_FREQUENCY YIN_SAMPLING_RATE   // audio sampling (ADC conversion) speed; clock speed will be half this
#define PLAYBACK_BUFFER_SIZE 4096          // must be power of 2; shorter limits possible latency, but longer crosses over itself less frequently (which produces a 'pop' sound)
  // 8192 gives 10s between pops at 98.21% speed (448hz tone vs 440)
  // 32768 gives 38s between pops at 98.21%
#define PLAYBACK_SPEED (long) (pow(2,32)/PLAYBACK_BUFFER_SIZE)
  // must be maxlong / PLAYBACK_BUFFER_SIZE, i.e. 2^17 for buffer size 2^15=32768
#define PLAYBACK_SHIFT (int) log2(PLAYBACK_SPEED) 
  // bits to shift output position by during playback to advance a single array cell
#define PLAYBACK_FASTEST (unsigned long) (PLAYBACK_SPEED*pow(2,32-PLAYBACK_SHIFT)-1)
  // maximum possible playback speed
#define ADC_FILTER 64                       // filter out deviations of less than this from a 0-4095 signal
#define YIN_ERROR 0.33                      // maximum allowed error to report a pitch instead of reporting -1
#define SIN_STEEP 2.0                       // steepness (1.0 to ~32.0) of sin function used to increase SNR, see https://www.desmos.com/calculator/wdtfsassev
#define c1 32.703f                          // Hz, lowest note we want to be able to detect


Yin yInMethod;
unsigned short steepSinTable[4096], conversions = 0;
short playbackData[PLAYBACK_BUFFER_SIZE], rawData[YIN_BUFFER_SIZE];
unsigned long playbackSpeed = PLAYBACK_SPEED;      // repitching speed, PLAYBACK_SPEED 1:1 playback, higher is faster
//bool clipping = false;
// Define the note names    0    1     2    3     4    5    6     7    8     9    10   11
const char* NoteNames[] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
// https://kagi.com/proxy/406c4d7753a67e92831ffdda0c432b53.jpg?c=5YigHpODNWx3qev9JrsT1PTQxfEZoqCu7AScTv9qhEVtkZGcr14jbzsSObe_h5RpyPZ_3SaH2D-plQ4JBBxXlE5SqoNz2y0_-pnDArkJWCSAj9V_-VEf8MSa9gXEda9B
//const int Chord[] = {1, 4, 8, 9}; // A Major 7th
const int Chord[] = {1, 2, 4, 6, 9}; // Dmaj9
//const int Chord[] = {0, 2, 5, 8}; // Fm
//const int Chord[] = {1, 3, 6, 10}; // Eb minor 7th
//const int Chord[] = {2, 5, 9, 10}; // Bb major 7th

#define noteLength sizeof(Chord) / sizeof(Chord[0]) * 8 //size of chord * 8
float noteTable[noteLength];

//#define DEBUG 1

void buildSteepSinTable();
void adc_setup();
void dac_setup();
void tc_setup();
void ADC_Handler();
float getNearestNoteFrequency(float frequency);
void buildNoteTable();
float binarySearchNotes(float f);

void setup() {
  Serial.begin(115200);
  buildSteepSinTable();
  buildNoteTable();
  noInterrupts();
  adc_setup();
  dac_setup();
  tc_setup();
  Yin_init(&yInMethod, YIN_ERROR);
  interrupts();
  #ifdef DEBUG
  Serial.println("Setup complete.");
  #endif
}

void loop() {
  static float frequency;
  while(!(ADC->ADC_ISR & ADC_ISR_EOC7)); // lock clock speed to half timer frequency
  if (conversions >= YIN_BUFFER_SIZE) {
    #ifdef DEBUG
    static unsigned int t;
    t = millis();
    #endif
    frequency = Yin_getPitch(&yInMethod, rawData); //at least 25 of 26ms is spent here
    conversions = 0;
    
    if (frequency >= c1 && frequency < 3600) { // Was a pitch actually detected?
      #ifdef DEBUG
      Serial.print("Frequency: ");
      Serial.print(frequency);
      #endif
      static float newFreq = frequency;
      newFreq = (newFreq + frequency) / 2.0f;
      frequency = newFreq;

      // Work out what note to tune to
      float targetFrequency = getNearestNoteFrequency(frequency);

      // Using this, we then adjust the playbackSpeed value
      playbackSpeed = constrain(round(targetFrequency * PLAYBACK_SPEED / frequency), 1, PLAYBACK_FASTEST); //was +ourSpeed
      #ifdef DEBUG
      Serial.print("    Playback speed: ");
      float pbs = 100.0*playbackSpeed/PLAYBACK_SPEED;
      Serial.print(pbs);
      Serial.print("%    Took ");
      Serial.print(millis() - t);
      Serial.print("ms");
      Serial.println("");
    #endif
    }
  }

  // if (clipping) {
  //     Serial.println(" ***************** CLIPPING ***************** ");
  //     clipping = false;
  //   }
}

void buildNoteTable() {
  uint8_t csize = sizeof(Chord) / sizeof(Chord[0]);
  uint8_t nearestSemitoneFromC1;

  for (uint8_t i=0; i<noteLength; i++) {
    nearestSemitoneFromC1 = Chord[i%csize] + 12*(i/csize);
    noteTable[i] = pow(2, nearestSemitoneFromC1 / 12.0f) * c1;
    //Serial.println(noteTable[i]);
  }
}

float binarySearchNotes(float f) {
    int first = 0;
    int last = noteLength-1;
    int mid = 0;
    do
    {
        mid = first + (last - first) / 2;
        if (f > noteTable[mid])
            first = mid + 1;
        else
            last = mid - 1;
        if (noteTable[mid] == f)
            return mid;
    } while (first <= last);

    if ((f - noteTable[mid]) < (noteTable[mid+1] - f)) {
      return noteTable[mid];
    } else {
      return noteTable[mid+1];
    }
}

// noteName can be NULL or a char* with space for 3 characvters + null terminator
float getNearestNoteFrequency(float frequency) {
  static char* noteName = new char[4];

  // Calculate the number of semitones from C1 (used as a reference point)
  //float nearestSemitoneFromC1f = 12.0 * log2(frequency / c1);
  float nearestSemitoneFromC1f = 12.0 * log2(binarySearchNotes(frequency) / c1);
  uint16_t nearestSemitoneFromC1 = round(nearestSemitoneFromC1f);
  
  // Calculate the detected note 
  if (noteName) {
    uint8_t noteNumber = nearestSemitoneFromC1 % 12;
    strcpy(noteName, NoteNames[noteNumber]);
    // Add the Octave
    uint8_t octave = 1 + (nearestSemitoneFromC1 / 12);
    char tmp[4];
    itoa(octave, tmp, 10);
    strcat(noteName, tmp);

    #ifdef DEBUG
    Serial.print("    Nearest Note: ");
    Serial.print(noteName);
    Serial.print(" (");
    Serial.print(pow(2, nearestSemitoneFromC1 / 12.0f) * c1);
    Serial.print(" Hz)");
    #endif 
  }

  // noteDif gives us -0.5 to 0.5 of how in-tune we are.
  float noteDiff = nearestSemitoneFromC1 - nearestSemitoneFromC1f;

  return pow(2, nearestSemitoneFromC1 / 12.0f) * c1;
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
  static short adc = 0;
  
  adc = ADC->ADC_CDR[7];                    // Reading ADC->ADC_CDR[i] clears EOCi bit
  if (conversions < YIN_BUFFER_SIZE) {
    rawData[conversions] = adc-2048;
    conversions++;
  }

  adc = steepSinTable[adc];
  if (abs(adc-2048) < ADC_FILTER) adc = 2048;
  //if (abs(adc-2048) == 2048) clipping = true;
  playbackData[inputPosition] = adc;
  inputPosition = (inputPosition+1) & (PLAYBACK_BUFFER_SIZE-1); 
  outputPosition += playbackSpeed; //output position will be bit-shifted by default playback speed so it advances only 1 at normal speed

  //Play from a different part
  DACC->DACC_CDR = playbackData[outputPosition >> PLAYBACK_SHIFT];
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