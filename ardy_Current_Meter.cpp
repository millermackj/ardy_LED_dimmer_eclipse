// Do not remove the include below
#include "ardy_Current_meter.h"
#include <stdio.h> // for string formatting

/*12V LED strip dimmer*/
#define LED_PWM_PIN 11 // digital output pin
#define POT_PIN 0 // analog input pin
#define CURRENT_PIN 2 // analog current input
#define SAMPLE_PERIOD 1 // A/D sampling period in milliseconds
#define INTERRUPTER_PIN 2 // tachometer pin
#define LED_PIN 13 // onboard LED

// Adafruit libraries for use of LCD display
#include "Adafruit_GFX.h"
#include "Adafruit_PCD8544.h"

// digital pins for LCD display
#define LCD_LED 9 	// pin 9 - LED backlight
#define LCD_SCLK 8	// pin 8 - Serial clock out (SCLK)
#define LCD_DIN 7		// pin 7 - Serial data out (DIN)
#define LCD_DC 6 		// pin 6 - Data/Command select (D/C)
#define LCD_CS 4		// pin 4 - LCD chip select (CS)
#define LCD_RST 5		// pin 5 - LCD reset (RST)

const double pi = 3.141592654;

long int dimmer_cts_new = 0;
long int dimmer_cts = 0;
int dimmer_debounce = 0;
int dimmer_debounce_limit = 20;
long int dimmer_pct = 0;
long unsigned int blink_clock = 0;
long unsigned int calc_tach_clock = 0;
long unsigned int sample_clock = 0;
long unsigned int update_display_clock = 0;
long unsigned int post_to_serial_clock = 0;
int update_display_period = 200;
int calc_tach_period = 50;
int post_to_serial_period = 100;
int led_state = 0;
int blink_period = 1000;
const int impulse_num_samples = 100;
int impulse_array[impulse_num_samples];
pulser_struct pulser;
int hold_display = 0;

double alpha = .1;
char outputbuffer[128];

// instantiate LCD Display controller using the Adafruit library
Adafruit_PCD8544 display = Adafruit_PCD8544(LCD_SCLK, LCD_DIN, LCD_DC, LCD_CS, LCD_RST);
encoder_struct tach_enc;
current_sensor_struct current_sensor;

void setup() {
	Serial.begin(115200);
	Serial.write("<h>time\tspeed</h>\n");
	Serial.write("<u>ms\trpm</u>\n");
  pinMode(LED_PWM_PIN, OUTPUT);
  //pinMode(POT_PIN, INPUT);
  pinMode(INTERRUPTER_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  display.setTextSize(1);
  display.setTextColor(BLACK);

  blink_clock = millis();

  tach_enc.max_hertz = 5;
  tach_enc.max_period = 5000;
  tach_enc.pin_num = INTERRUPTER_PIN;
  tach_enc.debounce_min = 5;
  tach_enc.alpha = 800;
  tach_enc.cts_per_rev = 1;
  tach_enc.hertz = 0;

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

	if (dispNumMill >= 100000) { // if number is still bigger than a hundred, remove decimal point
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

void poll_potentiometer(){
  dimmer_cts_new = (analogRead(POT_PIN)/5) * 5;
  if (dimmer_cts_new != dimmer_cts && dimmer_debounce < dimmer_debounce_limit){
  	dimmer_debounce++;
  	if (dimmer_debounce == dimmer_debounce_limit){
  		dimmer_cts = dimmer_cts_new;
  		dimmer_debounce = 0;
  	}
  }
  else{ // reset debounce count
  	dimmer_debounce = 0;
  }
  // calculate dimmer percentage to a tenth of a percent
  dimmer_pct = map(dimmer_cts, 0 , 1025, 0, 1000);
//  	pulser.pulse_up_period = (int)map(round(dimmer_cts), 0 , 873, 0, (long int)pulser.carrier_period);
}

void step_pulser(long unsigned int current_time, pulser_struct* pulser){
  if(current_time >= pulser->carrier_clock){
    	pulser->pulse_down_period = pulser->carrier_period - pulser->pulse_up_period;
      pulser->step_period_up = pulser->pulse_up_period / impulse_num_samples;
      pulser->step_period_down = pulser->pulse_down_period / impulse_num_samples;
      pulser->carrier_clock = current_time + pulser->carrier_period;
    	pulser->index = 0;
    	pulser->step_clock = current_time + pulser->offset_time;
    	pulser->down_or_up = UP;
    }
//    analogWrite(LED_PWM_PIN, impulse_array[pulser->index]);
//    showLED((long int)impulse_array[pulser->index] * 1000, 4);

    if(current_time >= pulser->step_clock){
    	if (pulser->down_or_up == UP && pulser->index < impulse_num_samples - 1) {
    		pulser->step_clock += pulser->step_period_up;
    		pulser->index++;
    	}
    	else if(pulser->index > 0){
    		pulser->down_or_up = DOWN;
    		pulser->step_clock += pulser->step_period_down;
    		pulser->index--;
    	}
    }

}

void poll_encoder(long unsigned int current_time, encoder_struct* encoder){
  if(!digitalRead(encoder->pin_num)){
    if(encoder->debounce_cnt < encoder->debounce_min)
    	encoder->debounce_cnt++;
    else{ // debounce count is equal to or greater than minimum
    	encoder->pin_state = HIGH;
    }
  }
  else{
    if(encoder->debounce_cnt > 0)
    	encoder->debounce_cnt--;
    else{ // debounce count is at zero
    	encoder->pin_state = LOW;
    }
  }

  if (encoder->last_state != encoder->pin_state){
  	digitalWrite(LED_PIN, encoder->pin_state);
  	if(encoder->pin_state == LOW){ // detect falling edge
//    		tach_enc->last_period = (tach_enc->alpha*(current_time - tach_enc->startTime)
//    				+ (1000L - tach_enc->alpha)*tach_enc->last_period)/1000;
  		encoder->last_period = (current_time - encoder->startTime);
  		encoder->startTime = current_time;
  	}
  }
  encoder->last_state = encoder->pin_state;
}

void calculate_tach(long unsigned int current_time, encoder_struct* tach_enc){
	if(tach_enc->last_period != 0){
		if((current_time - tach_enc->startTime) > tach_enc->max_period){
			tach_enc->hertz = 0;
		}
		else if(current_time - tach_enc->startTime > tach_enc->last_period)
			tach_enc->hertz = (1000L*1000L)/((current_time - tach_enc->startTime)*tach_enc->cts_per_rev);
		else{
			tach_enc->hertz = (1000L*1000L)/(tach_enc->last_period*tach_enc->cts_per_rev);
		}
	}
}

void calculate_current(current_sensor_struct* current_sense){
	current_sense->sense_cts = analogRead(CURRENT_PIN);
	current_sense->current = map(current_sense->sense_cts, 0, 1023, -2500, 2500);
}

void loop() {
	long unsigned int current_time = millis();
  if(current_time >= sample_clock){
  	// reset sampling clock
  	sample_clock += SAMPLE_PERIOD;

  	// poll the potentiometer once each sample period
  	poll_potentiometer();

  	// dim the LED string using potentiometer as a reference
    analogWrite(LED_PWM_PIN, map(dimmer_cts, 0, 1025, 0, 255));

    // step through impulse routine
    step_pulser(current_time, &pulser);

    // poll the tachometer encoder
    poll_encoder(current_time, &tach_enc);

		if(current_time >= calc_tach_clock){
			calc_tach_clock = current_time + calc_tach_period;
			calculate_tach(current_time, &tach_enc);
		}

//  if(current_time >= blink_clock){
//  	led_state = !led_state;
//  	digitalWrite(LED_PIN, led_state);
//  	blink_clock = current_time + blink_period;
//  }

		if(current_time >= update_display_clock){

			update_display_clock = current_time + update_display_period;
		  display.setCursor(0,0);
		  display.print("Current: ");
		  display.println(current_sensor.current);
		  display.print("Speed: ");
		  display.println(60*tach_enc.hertz / 1000.0);
		  display.display();
		}
		if(current_time >= post_to_serial_clock){
			post_to_serial_clock = current_time + post_to_serial_period;
//			Serial.write("<r>");
//			Serial.write(tach_enc.hertz / 1000);
//			Serial.write(".");
//			Serial.write(tach_enc.hertz % 1000);
			sprintf(outputbuffer, "<r>%lu.%03lu\t%ld.%03ld</r>\n", current_time/1000,
					current_time % 1000, (60*tach_enc.hertz)/1000, (60*tach_enc.hertz) % 1000);
			Serial.write(outputbuffer);
		}
  }
}
