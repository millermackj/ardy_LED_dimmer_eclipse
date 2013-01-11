// Do not remove the include below
#include "ardy_LED_dimmer_eclipse.h"


/*12V LED strip dimmer*/
#define LED_PWM_PIN 11 // digital output pin
#define POT_PIN 0 // analog input pin
#define SAMPLE_PERIOD 1 // A/D sampling period in milliseconds
#define INTERRUPTER_PIN 7
#define LED_PIN 8
// digital pins for LED Display driver MAX7221
#define LED_CLOCK_PIN 2
#define LED_DATA_PIN 3
#define LED_CSLOAD_PIN 4
#define UP 1
#define DOWN 0

#include <LedControl.h>
// digital pins for LED Display driver MAX7221
const int LEDClkPin = 2;
const int LEDDataPin = 3;
const int LEDCSLoadPin = 4;

const double pi = 3.141592654;

double dimmer_cts = 0;
int dimmer_pct = 0;
long unsigned int blink_clock = 0;
long unsigned int calc_cadence_clock = 0;
long unsigned int sample_clock = 0;
long unsigned int update_display_clock = 0;
int update_display_period = 200;
int calc_cadence_period = 50;
int led_state = 0;
int blink_period = 1000;
const int impulse_num_samples = 100;
int impulse_array[impulse_num_samples];
pulser_struct pulser;
int hold_display = 0;

double alpha = .05;
// instantiate LED Display controller -- the 4th argument is the number of displays
LedControl ledDisplay = LedControl(LED_DATA_PIN, LED_CLOCK_PIN, LED_CSLOAD_PIN, 1);
encoder_struct cadence_enc;

void setup() {
  pinMode(LED_PWM_PIN, OUTPUT);
  //pinMode(POT_PIN, INPUT);
  pinMode(INTERRUPTER_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  initLedDisp();
  blink_clock = millis();
  cadence_enc.max_hertz = 5;
  cadence_enc.max_period = 3000;
  cadence_enc.pin_num = INTERRUPTER_PIN;
  cadence_enc.debounce_min = 5;
  cadence_enc.alpha = 800;
  cadence_enc.cts_per_rev = 1;

  pulser.carrier_clock = 0;
  pulser.carrier_period = 5000;
  pulser.step_clock = 0;
  pulser.pulse_up_period = 1000;
  pulser.pulse_down_period = 3500;
  pulser.step_period_up = pulser.pulse_up_period / impulse_num_samples;
  pulser.step_period_down = pulser.pulse_down_period / impulse_num_samples;
  pulser.index = 0;
  pulser.offset_time = 0; // start pulse at beginning of each carrier period

  init_impulse(impulse_array, impulse_num_samples, 255);
}

void init_impulse(int * array, int array_length, int peak_value){
	int i = 0;
	for(i = 0; i < array_length; i ++){
//		array[i] = (int)((double)peak_value*0.5*(cos((2.0*pi*(double)i)/(double)array_length - pi) + 1.0));
		array[i] = -(int)((double)peak_value*0.5*(cos((pi/(double)array_length)*(double)i) - 1.0));
	}
}

// a little setup code for the LED display
void initLedDisp() {
	ledDisplay.shutdown(0, false);
	ledDisplay.setIntensity(0, 12);
	ledDisplay.clearDisplay(0);
}

// method that takes a number in millis and displays it to the hundredth on LED display
void showLED(long dispNumMill, int sigfigs) {

	// flag for negative value -- light up last decimal if negative
	bool neg = (dispNumMill < 0);

	int decipoint = 2; // decimal point after third digit from right by default

	dispNumMill = abs(dispNumMill);

	if (dispNumMill >= 100000) { // if number is bigger than a hundred, move decimal to right.
		decipoint = 1;
		dispNumMill = dispNumMill / 10;
	}

	if (dispNumMill >= 100000) { // if number is still bigger than a hundred, move decimal to right.
		decipoint = -1;
		dispNumMill = dispNumMill / 10;
	}

	int remainder = dispNumMill % 10;
	// do some rounding if necessary

	if (remainder >= 5)
		dispNumMill = dispNumMill - remainder + 10;

	int dig[4]; // array to store display digits

	// determine digit values by extracting them from dispNumMill

	dig[3] = (dispNumMill % 100000) / 10000;
	dig[2] = (dispNumMill % 10000) / 1000;
	dig[1] = (dispNumMill % 1000) / 100;
	dig[0] = (dispNumMill % 100) / 10;

	for (int i = 0; i < 4 - sigfigs; i++) {
		dig[i] = 0;
	}

	for (int i = 0; i < 4; i++) { // send digits to the display; light up rightmost DP if number is negative
		ledDisplay.setDigit(0, i, dig[i], (i == decipoint || (i == 0 && neg))); // set decimal point
	}
}

void loop() {
	long unsigned int current_time = millis();
  if(current_time >= sample_clock){
  	sample_clock += SAMPLE_PERIOD;
    dimmer_cts = alpha*analogRead(POT_PIN) + (1.0-alpha)*dimmer_cts;
    dimmer_pct = map(round(dimmer_cts), 0 , 873, 0, 100);
//  	pulser.pulse_up_period = (int)map(round(dimmer_cts), 0 , 873, 0, (long int)pulser.carrier_period);
    analogWrite(LED_PWM_PIN, map(round(dimmer_cts), 0, 873, 0, 255));
    if(current_time >= pulser.carrier_clock){
    	pulser.pulse_down_period = pulser.carrier_period - pulser.pulse_up_period;
      pulser.step_period_up = pulser.pulse_up_period / impulse_num_samples;
      pulser.step_period_down = pulser.pulse_down_period / impulse_num_samples;
      pulser.carrier_clock = current_time + pulser.carrier_period;
    	pulser.index = 0;
    	pulser.step_clock = current_time + pulser.offset_time;
    	pulser.down_or_up = UP;
    }
//    analogWrite(LED_PWM_PIN, impulse_array[pulser.index]);
//    showLED((long int)impulse_array[pulser.index] * 1000, 4);

    if(current_time >= pulser.step_clock){
    	if (pulser.down_or_up == UP && pulser.index < impulse_num_samples - 1) {
    		pulser.step_clock += pulser.step_period_up;
    		pulser.index++;
    	}
    	else if(pulser.index > 0){
    		pulser.down_or_up = DOWN;
    		pulser.step_clock += pulser.step_period_down;
    		pulser.index--;
    	}
    }


//    showLED(100L*dimmer_pct, 4);
    if(!digitalRead(cadence_enc.pin_num)){
      if(cadence_enc.debounce_cnt < cadence_enc.debounce_min)
      	cadence_enc.debounce_cnt++;
      else{ // debounce count is equal to or greater than minimum
      	cadence_enc.pin_state = HIGH;
      }
    }
    else{
      if(cadence_enc.debounce_cnt > 0)
      	cadence_enc.debounce_cnt--;
      else{ // debounce count is at zero
      	cadence_enc.pin_state = LOW;
      }
    }

    if (cadence_enc.last_state != cadence_enc.pin_state){
    	digitalWrite(LED_PIN, cadence_enc.pin_state);
    	if(cadence_enc.pin_state == LOW){ // detect falling edge
//    		cadence_enc.last_period = (cadence_enc.alpha*(current_time - cadence_enc.startTime)
//    				+ (1000L - cadence_enc.alpha)*cadence_enc.last_period)/1000;
    		cadence_enc.last_period = (current_time - cadence_enc.startTime);
    		cadence_enc.startTime = current_time;
    	}
    }
    cadence_enc.last_state = cadence_enc.pin_state;
  }
  if(current_time >= calc_cadence_clock){
  	if((current_time - cadence_enc.startTime) > cadence_enc.max_period){
  		cadence_enc.hertz = 0;
  	}
  	else if(current_time - cadence_enc.startTime > cadence_enc.last_period)
  		cadence_enc.hertz = (1000L*1000L)/(current_time - cadence_enc.startTime);
  	else{
  		cadence_enc.hertz = (1000L*1000L)/cadence_enc.last_period;
  	}
  	if(!hold_display){
  		showLED(60*cadence_enc.hertz,4);
  	}
  	else
  		hold_display--;
  	calc_cadence_clock = current_time+calc_cadence_period;
  }
//  if(current_time >= blink_clock){
//  	led_state = !led_state;
//  	digitalWrite(LED_PIN, led_state);
//  	blink_clock = current_time + blink_period;
//  }

  if(current_time >= update_display_clock){
  	update_display_clock = current_time + update_display_period;
//    showLED((long int)pulser.pulse_up_period * 1000, 4);
//    showLED((long int)dimmer_pct * 1000, 4);

  }
}
