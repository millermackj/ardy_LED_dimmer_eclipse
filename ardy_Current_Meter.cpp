// Do not remove the include below
#include "ardy_Current_meter.h"
#include <stdio.h> // for string formatting

/*12V LED strip dimmer*/
#define LED_PWM_PIN 11 // digital output pin
#define VOLT_PIN 1 // analog voltage input pin
#define CURRENT_PIN 3 // analog current input
#define SAMPLE_PERIOD 1 // sampling period in milliseconds
#define INTERRUPTER_PIN 2 // tachometer pin
#define LED_PIN 13 // onboard LED
#define BTN_PIN 12 // input button

// digital pins for LCD display
#define BACKLIGHT_PIN 9 	// pin 9 - LED backlight

#define PIN_SCE   4	// pin 4 - LCD chip select (CS)
#define PIN_RESET 5	// pin 5 - LCD reset (RST)
#define PIN_DC    6	// pin 6 - Data/Command select (D/C)
#define PIN_SDIN  7 // pin 7 - Serial data out (DIN)
#define PIN_SCLK  8	// pin 8 - Serial clock out (SCLK)

#define LCD_C     LOW 	// LCD command
#define LCD_D     HIGH	// LCD data

#define LCD_X     84
#define LCD_Y     48

static const byte ASCII[][5] =
{
 {0x00, 0x00, 0x00, 0x00, 0x00} // 20
,{0x00, 0x00, 0x5f, 0x00, 0x00} // 21 !
,{0x00, 0x07, 0x00, 0x07, 0x00} // 22 "
,{0x14, 0x7f, 0x14, 0x7f, 0x14} // 23 #
,{0x24, 0x2a, 0x7f, 0x2a, 0x12} // 24 $
,{0x23, 0x13, 0x08, 0x64, 0x62} // 25 %
,{0x36, 0x49, 0x55, 0x22, 0x50} // 26 &
,{0x00, 0x05, 0x03, 0x00, 0x00} // 27 '
,{0x00, 0x1c, 0x22, 0x41, 0x00} // 28 (
,{0x00, 0x41, 0x22, 0x1c, 0x00} // 29 )
,{0x14, 0x08, 0x3e, 0x08, 0x14} // 2a *
,{0x08, 0x08, 0x3e, 0x08, 0x08} // 2b +
,{0x00, 0x50, 0x30, 0x00, 0x00} // 2c ,
,{0x08, 0x08, 0x08, 0x08, 0x08} // 2d -
,{0x00, 0x60, 0x60, 0x00, 0x00} // 2e .
,{0x20, 0x10, 0x08, 0x04, 0x02} // 2f /
,{0x3e, 0x51, 0x49, 0x45, 0x3e} // 30 0
,{0x00, 0x42, 0x7f, 0x40, 0x00} // 31 1
,{0x42, 0x61, 0x51, 0x49, 0x46} // 32 2
,{0x21, 0x41, 0x45, 0x4b, 0x31} // 33 3
,{0x18, 0x14, 0x12, 0x7f, 0x10} // 34 4
,{0x27, 0x45, 0x45, 0x45, 0x39} // 35 5
,{0x3c, 0x4a, 0x49, 0x49, 0x30} // 36 6
,{0x01, 0x71, 0x09, 0x05, 0x03} // 37 7
,{0x36, 0x49, 0x49, 0x49, 0x36} // 38 8
,{0x06, 0x49, 0x49, 0x29, 0x1e} // 39 9
,{0x00, 0x36, 0x36, 0x00, 0x00} // 3a :
,{0x00, 0x56, 0x36, 0x00, 0x00} // 3b ;
,{0x08, 0x14, 0x22, 0x41, 0x00} // 3c <
,{0x14, 0x14, 0x14, 0x14, 0x14} // 3d =
,{0x00, 0x41, 0x22, 0x14, 0x08} // 3e >
,{0x02, 0x01, 0x51, 0x09, 0x06} // 3f ?
,{0x32, 0x49, 0x79, 0x41, 0x3e} // 40 @
,{0x7e, 0x11, 0x11, 0x11, 0x7e} // 41 A
,{0x7f, 0x49, 0x49, 0x49, 0x36} // 42 B
,{0x3e, 0x41, 0x41, 0x41, 0x22} // 43 C
,{0x7f, 0x41, 0x41, 0x22, 0x1c} // 44 D
,{0x7f, 0x49, 0x49, 0x49, 0x41} // 45 E
,{0x7f, 0x09, 0x09, 0x09, 0x01} // 46 F
,{0x3e, 0x41, 0x49, 0x49, 0x7a} // 47 G
,{0x7f, 0x08, 0x08, 0x08, 0x7f} // 48 H
,{0x00, 0x41, 0x7f, 0x41, 0x00} // 49 I
,{0x20, 0x40, 0x41, 0x3f, 0x01} // 4a J
,{0x7f, 0x08, 0x14, 0x22, 0x41} // 4b K
,{0x7f, 0x40, 0x40, 0x40, 0x40} // 4c L
,{0x7f, 0x02, 0x0c, 0x02, 0x7f} // 4d M
,{0x7f, 0x04, 0x08, 0x10, 0x7f} // 4e N
,{0x3e, 0x41, 0x41, 0x41, 0x3e} // 4f O
,{0x7f, 0x09, 0x09, 0x09, 0x06} // 50 P
,{0x3e, 0x41, 0x51, 0x21, 0x5e} // 51 Q
,{0x7f, 0x09, 0x19, 0x29, 0x46} // 52 R
,{0x46, 0x49, 0x49, 0x49, 0x31} // 53 S
,{0x01, 0x01, 0x7f, 0x01, 0x01} // 54 T
,{0x3f, 0x40, 0x40, 0x40, 0x3f} // 55 U
,{0x1f, 0x20, 0x40, 0x20, 0x1f} // 56 V
,{0x3f, 0x40, 0x38, 0x40, 0x3f} // 57 W
,{0x63, 0x14, 0x08, 0x14, 0x63} // 58 X
,{0x07, 0x08, 0x70, 0x08, 0x07} // 59 Y
,{0x61, 0x51, 0x49, 0x45, 0x43} // 5a Z
,{0x00, 0x7f, 0x41, 0x41, 0x00} // 5b [
,{0x02, 0x04, 0x08, 0x10, 0x20} // 5c ¥
,{0x00, 0x41, 0x41, 0x7f, 0x00} // 5d ]
,{0x04, 0x02, 0x01, 0x02, 0x04} // 5e ^
,{0x40, 0x40, 0x40, 0x40, 0x40} // 5f _
,{0x00, 0x01, 0x02, 0x04, 0x00} // 60 `
,{0x20, 0x54, 0x54, 0x54, 0x78} // 61 a
,{0x7f, 0x48, 0x44, 0x44, 0x38} // 62 b
,{0x38, 0x44, 0x44, 0x44, 0x20} // 63 c
,{0x38, 0x44, 0x44, 0x48, 0x7f} // 64 d
,{0x38, 0x54, 0x54, 0x54, 0x18} // 65 e
,{0x08, 0x7e, 0x09, 0x01, 0x02} // 66 f
,{0x0c, 0x52, 0x52, 0x52, 0x3e} // 67 g
,{0x7f, 0x08, 0x04, 0x04, 0x78} // 68 h
,{0x00, 0x44, 0x7d, 0x40, 0x00} // 69 i
,{0x20, 0x40, 0x44, 0x3d, 0x00} // 6a j
,{0x7f, 0x10, 0x28, 0x44, 0x00} // 6b k
,{0x00, 0x41, 0x7f, 0x40, 0x00} // 6c l
,{0x7c, 0x04, 0x18, 0x04, 0x78} // 6d m
,{0x7c, 0x08, 0x04, 0x04, 0x78} // 6e n
,{0x38, 0x44, 0x44, 0x44, 0x38} // 6f o
,{0x7c, 0x14, 0x14, 0x14, 0x08} // 70 p
,{0x08, 0x14, 0x14, 0x18, 0x7c} // 71 q
,{0x7c, 0x08, 0x04, 0x04, 0x08} // 72 r
,{0x48, 0x54, 0x54, 0x54, 0x20} // 73 s
,{0x04, 0x3f, 0x44, 0x40, 0x20} // 74 t
,{0x3c, 0x40, 0x40, 0x20, 0x7c} // 75 u
,{0x1c, 0x20, 0x40, 0x20, 0x1c} // 76 v
,{0x3c, 0x40, 0x30, 0x40, 0x3c} // 77 w
,{0x44, 0x28, 0x10, 0x28, 0x44} // 78 x
,{0x0c, 0x50, 0x50, 0x50, 0x3c} // 79 y
,{0x44, 0x64, 0x54, 0x4c, 0x44} // 7a z
,{0x00, 0x08, 0x36, 0x41, 0x00} // 7b {
,{0x00, 0x00, 0x7f, 0x00, 0x00} // 7c |
,{0x00, 0x41, 0x36, 0x08, 0x00} // 7d }
,{0x10, 0x08, 0x08, 0x10, 0x08} // 7e ←
,{0x78, 0x46, 0x41, 0x46, 0x78} // 7f →
};


const double pi = 3.141592654;

#define UP 1
#define DOWN 0

#define SHOW_WATTS 0
#define SHOW_RPM 1

long int watts = 0;
long int dimmer_cts_new = 0;
long int dimmer_cts = 0;
int dimmer_debounce = 0;
int dimmer_debounce_limit = 20;
long int dimmer_pct = 0;
long unsigned int blink_clock = 0;
long unsigned int calc_tach_clock = 0;
long unsigned int sample_clock = 0;
long unsigned int calc_current_clock = 0;
long unsigned int update_display_clock = 0;
long unsigned int post_to_serial_clock = 0;
long unsigned int toggle_display_clock = 0;
long unsigned int backlight_clock = 2000;
long unsigned int update_tach_clock = 0;

int update_display_period = 1000;
int calc_tach_period = 10;
int calc_current_period = 4;
int post_to_serial_period = 100;
int led_state = 0;
int blink_period = 1000;
int toggle_display_period = 2000;
int backlight_period = 7000;
int update_tach_period = 5;


// some pushbutton related constants
const unsigned int shortPress = 20; // length of button press in milliseconds
const unsigned int longPress = 1000;
const unsigned int xlongPress = 2000;
const int btnUp = LOW;
const int btnDown = HIGH;


//const int impulse_num_samples = 100;
//int impulse_array[impulse_num_samples];
//pulser_struct pulser;
int hold_display = 0;
int toggle_display = SHOW_RPM;

double alpha = .1;
char outputbuffer[128];

encoder_struct tach_enc;
current_sensor_struct current_sensor;
volt_sensor_struct volt_sensor;
button_struct button;

void LcdCharacter(char character)
{
  LcdWrite(LCD_D, 0x00);
  for (int index = 0; index < 5; index++)
  {
    LcdWrite(LCD_D, ASCII[character - 0x20][index]);
  }
  LcdWrite(LCD_D, 0x00);
}

void LcdClear(void)
{
  for (int index = 0; index < LCD_X * LCD_Y / 8; index++)
  {
    LcdWrite(LCD_D, 0x00);
  }
}

void LcdInitialize(void)
{
  pinMode(PIN_SCE, OUTPUT);
  pinMode(PIN_RESET, OUTPUT);
  pinMode(PIN_DC, OUTPUT);
  pinMode(PIN_SDIN, OUTPUT);
  pinMode(PIN_SCLK, OUTPUT);
  digitalWrite(PIN_RESET, LOW);
  digitalWrite(PIN_RESET, HIGH);
  LcdWrite(LCD_C, 0x21 );  // LCD Extended Commands.
  LcdWrite(LCD_C, 0xB1 );  // Set LCD Vop (Contrast).
  LcdWrite(LCD_C, 0x04 );  // Set Temp coefficent. //0x04
  LcdWrite(LCD_C, 0x14 );  // LCD bias mode 1:48. //0x13
  LcdWrite(LCD_C, 0x0C );  // LCD in normal mode.
  LcdWrite(LCD_C, 0x20 );
  LcdWrite(LCD_C, 0x0C );
}

void LcdString(char *characters)
{
  while (*characters)
  {
    LcdCharacter(*characters++);
  }
}

void LcdWrite(byte dc, byte data)
{
  digitalWrite(PIN_DC, dc);
  digitalWrite(PIN_SCE, LOW);
  shiftOut(PIN_SDIN, PIN_SCLK, MSBFIRST, data);
  digitalWrite(PIN_SCE, HIGH);
}

void gotoXY(int x, int y) {
  LcdWrite(0, 0x80 | x);  // Column.
  LcdWrite(0, 0x40 | y);  // Row.  ?
}

void setup() {
	// set up the encoder interrupt to detect falling edge on digital pin 2
	attachInterrupt(0, encoder_interrupt, FALLING);
	Serial.begin(115200);
	Serial.write("<h>time\tspeed\tcurrent\tcurrent_sense\tvoltage\tvoltage_sense</h>\n");
	Serial.write("<u>ms\trpm\tmA\tcounts\tvolts\tcts</u>\n");
  pinMode(LED_PWM_PIN, OUTPUT);
  pinMode(VOLT_PIN, INPUT);
  pinMode(INTERRUPTER_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BTN_PIN, INPUT);

  LcdInitialize();
  gotoXY(0,0);
  LcdClear();
  LcdString((char*)"Current:");

  gotoXY(0,2);
  LcdString((char*)"Speed:");

  gotoXY(0,4);
  LcdString((char*)"Voltage:");

  blink_clock = millis();

  tach_enc.min_hertz = 5;
  tach_enc.max_period = 5000;
  tach_enc.pin_num = INTERRUPTER_PIN;
  tach_enc.debounce_min = 5;
  tach_enc.alpha = 800;
  tach_enc.cts_per_rev = 1;
  tach_enc.hertz = 0;

  button.button_pin = BTN_PIN;
  button.button_state = btnDown;

//  pulser.carrier_clock = 0;
//  pulser.carrier_period = 5000;
//  pulser.step_clock = 0;
//  pulser.pulse_up_period = 1000;
//  pulser.pulse_down_period = 3500;
//  pulser.step_period_up = pulser.pulse_up_period / impulse_num_samples;
//  pulser.step_period_down = pulser.pulse_down_period / impulse_num_samples;
//  pulser.index = 0;
//  pulser.offset_time = 0; // start pulse at beginning of each carrier period

  volt_sensor.max_volts = 68000; // mV at sensor saturation (depends on pot adj)

//  init_impulse(impulse_array, impulse_num_samples, 255);


}

void init_impulse(int * array, int array_length, int peak_value){
	int i = 0;
	for(i = 0; i < array_length; i ++){
//		array[i] = (int)((double)peak_value*0.5*(cos((2.0*pi*(double)i)/(double)array_length - pi) + 1.0));
		array[i] = -(int)((double)peak_value*0.5*(cos((pi/(double)array_length)*(double)i) - 1.0));
	}
}


//void poll_potentiometer(){
//  dimmer_cts_new = (analogRead(POT_PIN)/5) * 5;
//  if (dimmer_cts_new != dimmer_cts && dimmer_debounce < dimmer_debounce_limit){
//  	dimmer_debounce++;
//  	if (dimmer_debounce == dimmer_debounce_limit){
//  		dimmer_cts = dimmer_cts_new;
//  		dimmer_debounce = 0;
//  	}
//  }
//  else{ // reset debounce count
//  	dimmer_debounce = 0;
//  }
//  // calculate dimmer percentage to a tenth of a percent
//  dimmer_pct = map(dimmer_cts, 0 , 1025, 0, 1000);
////  	pulser.pulse_up_period = (int)map(round(dimmer_cts), 0 , 873, 0, (long int)pulser.carrier_period);
//}

//void step_pulser(long unsigned int current_time, pulser_struct* pulser){
//  if(current_time >= pulser->carrier_clock){
//    	pulser->pulse_down_period = pulser->carrier_period - pulser->pulse_up_period;
//      pulser->step_period_up = pulser->pulse_up_period / impulse_num_samples;
//      pulser->step_period_down = pulser->pulse_down_period / impulse_num_samples;
//      pulser->carrier_clock = current_time + pulser->carrier_period;
//    	pulser->index = 0;
//    	pulser->step_clock = current_time + pulser->offset_time;
//    	pulser->down_or_up = UP;
//    }
////    analogWrite(LED_PWM_PIN, impulse_array[pulser->index]);
////    showLED((long int)impulse_array[pulser->index] * 1000, 4);
//
//    if(current_time >= pulser->step_clock){
//    	if (pulser->down_or_up == UP && pulser->index < impulse_num_samples - 1) {
//    		pulser->step_clock += pulser->step_period_up;
//    		pulser->index++;
//    	}
//    	else if(pulser->index > 0){
//    		pulser->down_or_up = DOWN;
//    		pulser->step_clock += pulser->step_period_down;
//    		pulser->index--;
//    	}
//    }
//
//}

// interrupt called when tachometer pin goes from HIGH to LOW
void encoder_interrupt(){
	tach_enc.pin_state = 1;
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
		else if((current_time - tach_enc->startTime) > tach_enc->last_period)
			tach_enc->hertz = (1000L*1000L)/((current_time - tach_enc->startTime)*tach_enc->cts_per_rev);
		else{
			tach_enc->hertz = (1000L*1000L)/(tach_enc->last_period*tach_enc->cts_per_rev);
		}
	}
}

void calculate_current(current_sensor_struct* current_sense){
	current_sense->sense_cts = ((long int)(current_sense->sense_cts*800L) + (long int)((analogRead(CURRENT_PIN)-9)*200L))/1000L;
	current_sense->current = (int)map(current_sense->sense_cts, 0, 1023, -25000, 25000);
}

void calculate_voltage(volt_sensor_struct* volt_sensor){
	volt_sensor->sense_cts = ((long int)(volt_sensor->sense_cts*800L) + (long int)((analogRead(VOLT_PIN))*200L))/1000L;
	volt_sensor->voltage = map(volt_sensor->sense_cts, 0, 1023, 0, volt_sensor->max_volts);
}

int pollButton(button_struct* button) {

	int returnVal = 0; // default value means button was not down long enough to consider it a press
	int newState = digitalRead(button->button_pin); // poll for button state

	if (button->button_state != newState) { // only proceed if there is a change of state
		button->button_state = newState;
		if (newState == btnDown) { //the button has just been pressed down so start timing
			button->timeOfPress = millis();
		}

		else { // otherwise the button has just let up so decide what to do
			long unsigned pressDuration = millis() - button->timeOfPress; // length of time button has been down

			if (pressDuration >= xlongPress)
				returnVal = xlongPress;
			else if (pressDuration >= longPress) // if button is pressed for long time, return that info
				returnVal = longPress;
			else if (pressDuration >= shortPress)
				returnVal = shortPress;
		}
	}

	return returnVal;
}

void loop() {
	long unsigned int current_time = millis();
  if(current_time >= sample_clock){
  	// reset sampling clock
  	sample_clock += SAMPLE_PERIOD;

  	// poll the potentiometer once each sample period
//  	poll_potentiometer();

  	// dim the LED string using potentiometer as a reference
//    analogWrite(LED_PWM_PIN, map(dimmer_cts, 0, 1025, 0, 255));

    // step through impulse routine
//    step_pulser(current_time, &pulser);

    // poll the tachometer encoder
    //poll_encoder(current_time, &tach_enc);

    if (pollButton(&button) >= shortPress){
    	backlight_clock = current_time + backlight_period;
    }

    if(current_time >= update_tach_clock){
    	update_tach_clock = current_time + update_tach_period;
    	// check if tachometer interrupt has been fired
    	if(tach_enc.pin_state){
    		tach_enc.pin_state = 0;
    		tach_enc.last_period = (current_time - tach_enc.startTime);
    		tach_enc.startTime = current_time;
    	}
    	digitalWrite(LED_PIN, digitalRead(tach_enc.pin_num));
    }

		if(current_time >= calc_tach_clock){
			calc_tach_clock = current_time + calc_tach_period;
			calculate_tach(current_time, &tach_enc);
		}

		if(current_time >= calc_current_clock){
			calc_current_clock = current_time + calc_current_period;
			calculate_current(&current_sensor);
			calculate_voltage(&volt_sensor);
		}

//  if(current_time >= blink_clock){
//  	led_state = !led_state;
//  	digitalWrite(LED_PIN, led_state);
//  	blink_clock = current_time + blink_period;
//  }

		if(current_time >= toggle_display_clock){
			toggle_display_clock = current_time + toggle_display_period;
			toggle_display = !toggle_display;
		  gotoXY(0,2);
		  if(toggle_display == SHOW_RPM)
		  	LcdString((char*)"Speed:");
		  else if(toggle_display == SHOW_WATTS)
		  	LcdString((char*)"Power:");
		}

		if(current_time >= update_display_clock){

			update_display_clock = current_time + update_display_period;
		  snprintf(outputbuffer, 11,"%d.%02d Amps", (current_sensor.current)/1000,
		  		((abs(current_sensor.current) % 1000) / 10));

		  gotoXY(6,1);
		  LcdString((char*)"           ");
		  gotoXY(6,1);
		  LcdString(outputbuffer);


		  if(toggle_display == SHOW_RPM){
			  snprintf(outputbuffer,11,"%ld.%01ld RPM", (60*tach_enc.hertz)/1000, ((60*tach_enc.hertz)%1000) / 100);
		  }

		  else if(toggle_display == SHOW_WATTS){
		  	watts = abs(current_sensor.current * volt_sensor.voltage) / 1000;
			  snprintf(outputbuffer,11,"%ld.%01ld Watts", watts/1000, (watts%1000) / 100);
		  }

		  gotoXY(6,3);
		  LcdString((char*)"           ");
		  gotoXY(6,3);
		  LcdString(outputbuffer);

		  snprintf(outputbuffer,12,"%ld.%02ld Volts", volt_sensor.voltage/1000, ((volt_sensor.voltage)%1000) / 10);
		  gotoXY(6,5);
		  LcdString((char*)"           ");
		  gotoXY(6,5);
		  LcdString(outputbuffer);
		}
//		if(current_time >= post_to_serial_clock){
//			post_to_serial_clock = current_time + post_to_serial_period;
////			Serial.write("<r>");
////			Serial.write(tach_enc.hertz / 1000);
////			Serial.write(".");
////			Serial.write(tach_enc.hertz % 1000);
//			sprintf(outputbuffer, "<r>%lu.%03lu\t%ld.%03ld\t%d\t%ld\t%ld.%03ld\t%ld</r>\n", current_time/1000,
//					current_time % 1000, (60*tach_enc.hertz)/1000, ((60*tach_enc.hertz) % 1000)/10,current_sensor.current, current_sensor.sense_cts,
//					volt_sensor.voltage / 1000, volt_sensor.voltage % 1000, volt_sensor.sense_cts);
//			Serial.write(outputbuffer);
//		}

		if(current_time <= backlight_clock){
			analogWrite(BACKLIGHT_PIN, 35);
		}
		else
			analogWrite(BACKLIGHT_PIN, 0);
  }
}
