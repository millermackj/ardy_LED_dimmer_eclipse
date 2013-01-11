// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef ardy_LED_dimmer_eclipse_H_
#define ardy_LED_dimmer_eclipse_H_
#include "Arduino.h"
//add your includes for the project ardy_LED_dimmer_eclipse here


//end of add your includes here
#ifdef __cplusplus
extern "C" {
#endif
void loop();
void setup();
#ifdef __cplusplus
} // extern "C"
#endif

//add your function definitions for the project ardy_LED_dimmer_eclipse here
typedef struct{
	int debounce_cnt;
	int debounce_min;
  int pin_state;
	long unsigned int startTime;
  int max_hertz;
  long int hertz;
  int pin_num;
  int last_period;
  int cts_per_rev;
  unsigned int last_state;
  unsigned int max_period;
  long int alpha;
}encoder_struct;

typedef struct{
	int carrier_period;
	int pulse_up_period;
	int pulse_down_period;
	int step_period_up;
	int step_period_down;
	long unsigned int carrier_clock;
	long unsigned int pulse_clock;
	long unsigned int step_clock;
	int index;
	int offset_time; // ms after carrier to start pulse
	int down_or_up;

}pulser_struct;


void initLedDisp();
void showLED(long dispNumMill, int sigfigs);
int poll_encoder(encoder_struct encoder);
void init_impulse(int * array, int array_length, int peak_value);

//Do not add code below this line
#endif /* ardy_LED_dimmer_eclipse_H_ */
