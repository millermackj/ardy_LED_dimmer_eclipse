// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef ardy_Current_Meter_H_
#define ardy_Current_Meter_H_
#include "Arduino.h"
//add your includes for the project ardy_Current_Meter here


//end of add your includes here
#ifdef __cplusplus
extern "C" {
#endif
void loop();
void setup();
#ifdef __cplusplus
} // extern "C"
#endif

//add your function definitions for the project ardy_Current_Meter here
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
  int last_state;
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

typedef struct{
	int sense_pin_num;
	int supply_pin_num;
	long int sense_cts;
	int suppy_cts;
	long int current_ratio; // converts ratio of sense:2*supply to mA
	int current;
}current_sensor_struct;

typedef struct{
	int sense_pin_num;
	long int sense_cts;
	long int max_volts; // voltage when sensor reads 1023
	long int voltage;
}volt_sensor_struct;


void LcdCharacter(char character);
void LcdClear(void);
void LcdInitialize(void);
void LcdString(char *characters);
void LcdWrite(byte dc, byte data);
void gotoXY(int x, int y);
void initLedDisp();
void showLED(long dispNumMill, int sigfigs);
void poll_potentiometer();
void init_impulse(int * array, int array_length, int peak_value);
void step_pulser(long unsigned int current_ime, pulser_struct* pulse);
void poll_encoder(long unsigned int current_time, encoder_struct* encoder);
void calculate_tach(long unsigned int current_time, encoder_struct* cadence_enc);
void calculate_current(current_sensor_struct* current_sensor);
void calculate_voltage(volt_sensor_struct* volt_sensor);
//Do not add code below this line
#endif /* ardy_Current_Meter_H_ */
