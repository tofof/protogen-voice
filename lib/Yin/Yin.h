#ifndef Yin_h
#define Yin_h

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#define YIN_SAMPLING_RATE 48000

// #define YIN_BUFFER_SIZE 4096                      // audio buffer size
// #define HALF_BUFFER_SIZE 2048

// #define YIN_BUFFER_SIZE 3072                      // audio buffer size
// #define HALF_BUFFER_SIZE 1536

// #define YIN_BUFFER_SIZE 2048                      // audio buffer size
// #define HALF_BUFFER_SIZE 1024

// #define YIN_BUFFER_SIZE 1536                      // audio buffer size
// #define HALF_BUFFER_SIZE 768

#define YIN_BUFFER_SIZE 1280                      // audio buffer size
#define HALF_BUFFER_SIZE 640

// #define YIN_BUFFER_SIZE 960                      // audio buffer size
// #define HALF_BUFFER_SIZE 480

// #define YIN_BUFFER_SIZE 640                      // audio buffer size
// #define HALF_BUFFER_SIZE 320


/**
 * @struct  Yin
 * @breif	Object to encapsulate the parameters for the Yin pitch detection algorithm 
 */
typedef struct _Yin {
	float yinBuffer[HALF_BUFFER_SIZE];		/**< Buffer that stores the results of the intermediate processing steps of the algorithm */
	float threshold;		/**< Allowed uncertainty in the result as a decimal (i.e 0.15 is 15%) */
} Yin;

/**
 * Initialise the Yin pitch detection object
 * @param yin        Yin pitch detection object to initialise
 * @param threshold  Allowed uncertainty (e.g 0.05 will return a pitch with ~95% probability)
 */
void Yin_init(Yin *yin, float threshold);

/**
 * Runs the Yin pitch detection algortihm
 * @param  yin    Initialised Yin object
 * @param  buffer Buffer of samples to analyse
 * @return        Fundamental frequency of the signal in Hz. Returns -1 if pitch can't be found
 */
float Yin_getPitch(Yin *yin, int16_t* buffer);

/**
 * Certainty of the pitch found 
 * @param  yin Yin object that has been run over a buffer
 * @return     Returns the certainty of the note found as a decimal (i.e 0.3 is 30%)
 */
float Yin_getProbability(Yin *yin);
	


#endif
#ifdef __cplusplus
}
#endif