// Blinks the built-in LED

#include "wirish.h"
#include "usb.h"
#include "timer.h"

#define PWM_PIN_X 11
#define X_ENCODER_A 5 //PB6 Timer4_CH1
#define X_ENCODER_B 9 //PB7 Timer4_CH2
#define Y_ENCODER_A 6 //PA8 Timer1_CH1
#define Y_ENCODER_B 7 //PB9 Timer1_CH2
#define BUTTON1 21

#define X_STOP_1 10
#define X_STOP_2 2

#define STOP 32222
#define LOW_SPEED_LEFT (STOP + 3200)

//Modes
#define NUM_MODES 3
#define ALL_STOP 0
#define HOME 1
#define RUN 2

volatile int mode = 0;

//Position
volatile signed long current_x=0;
volatile signed long current_y=0;
signed short delta_x=0,delta_y=0;

//Control
unsigned short knob_val=0;
volatile unsigned short pwm_x=0;
volatile unsigned short pwm_y=0;

//Homing
bool XStop1 = 0; //First
bool XStop2 = 0; //Home
bool YStop1 = 0; //First
bool YStop2 = 0; //Home

//Debug
int toggle = 1;
int toggle2 = 0;
char indicator = '0';
bool button1State;
bool lastButton1State;
int lastDebounceTime = 0;
int debounceDelay = 100;

void homedX() {
	pwm_x = STOP;
	pwmWrite(PWM_PIN_X,pwm_x);
	
	current_x = 0;
	timer_set_count(TIMER4, 0);
	mode = STOP;
}

void homedY() {
	pwm_y = STOP;
	//pwmWrite(PWM_PIN_Y, pwm_y);
	
	current_y = 0;
	timer_set_count(TIMER1, 0);
	mode = STOP;
}

void setup() {

	//Potentiometer
    pinMode(3, INPUT_ANALOG);
    
    //Encoders
    pinMode(X_ENCODER_A, INPUT); 
    pinMode(X_ENCODER_B, INPUT);
    pinMode(Y_ENCODER_A, INPUT);
    pinMode(Y_ENCODER_B, INPUT);
    
    //PWM Setup
    pinMode(PWM_PIN_X,PWM);
    
    //Encoder setup
    timer_init(TIMER1);
    timer_init(TIMER4);
    timer_pause(TIMER1);
    timer_pause(TIMER4);
    timer_set_prescaler(TIMER1,0);
    timer_set_prescaler(TIMER4,0);
    (TIMER4->regs).gen->CCMR1 |= TIMER_CCMR1_CC2S;  //enable input2
    (TIMER4->regs).gen->CCMR1 |= TIMER_CCMR1_CC1S;  //enable input1
    (TIMER4->regs).gen->CCMR1 |= 0x3<<12; // IC2F Filter
    (TIMER4->regs).gen->CCMR1 |= 0x3<<4;  // IC1F Filter
    (TIMER4->regs).gen->CCER &= ~(TIMER_CCER_CC1P); //non-inverted polarity channel 1
    (TIMER4->regs).gen->CCER &= ~(TIMER_CCER_CC2P); //non-inverted polarity channel 2
    (TIMER4->regs).gen->SMCR |= TIMER_SMCR_SMS_ENCODER3;
    (TIMER1->regs).gen->CCMR1 |= TIMER_CCMR1_CC2S;
    (TIMER1->regs).gen->CCMR1 |= TIMER_CCMR1_CC1S;
    (TIMER1->regs).gen->CCMR1 |= 0x3<<12; // IC2F Filter
    (TIMER1->regs).gen->CCMR1 |= 0x3<<4;  // IC1F Filter
    (TIMER1->regs).gen->CCER &= ~(TIMER_CCER_CC1P);
    (TIMER1->regs).gen->CCER &= ~(TIMER_CCER_CC2P);
    (TIMER1->regs).gen->SMCR |= TIMER_SMCR_SMS_ENCODER3;
    timer_generate_update(TIMER1);
    timer_generate_update(TIMER4);
    timer_resume(TIMER1);
    timer_resume(TIMER4);   
        
    //Debug LEDs    
    pinMode(14, OUTPUT);
    pinMode(24, OUTPUT);
    pinMode(29, OUTPUT);
    digitalWrite(24,1);
    
    //Setup stops
    pinMode(X_STOP_1, INPUT);
    pinMode(X_STOP_2, INPUT);
    attachInterrupt(X_STOP_2, homedX, RISING);
    //attachInterrupt(Y_STOP_2, homedY, RISING);
    
    pinMode(BUTTON1, INPUT);
    
    SerialUSB.println("Hello");
}

void printStatus() {
	
	if (knob_val < 1900) {
    	indicator = 'L';
    } else if (knob_val > 2200) {
    	indicator = 'R';
    } else {
    	indicator = 'C';
    }
    
    if (mode == HOME) {
    	SerialUSB.print("HOME ");
    } else if (mode == RUN) {
    	SerialUSB.print("RUN  ");
    } else if (mode == ALL_STOP) {
    	SerialUSB.print("STOP ");
    } else {
    	SerialUSB.print("ERROR ");
    }
    SerialUSB.print(mode);
	SerialUSB.print(" X1:");
	SerialUSB.print(XStop1);
	SerialUSB.print(" X2:");
	SerialUSB.print(XStop2);
	
    SerialUSB.print(" KnobVal: ");
    SerialUSB.print(knob_val);
    SerialUSB.print(" ");
    SerialUSB.print(indicator);
    SerialUSB.print(" PWM: ");
    SerialUSB.print(pwm_x);
    SerialUSB.print(" X_pos: ");
    SerialUSB.print(current_x);
    
    //SerialUSB.print(" BUT1: ");
    //SerialUSB.print(button1State);
    
    SerialUSB.println();
}

void blinkLights() {
	digitalWrite(14, toggle);
    digitalWrite(24, toggle2);
    digitalWrite(29, toggle);
    toggle ^= 1;
    toggle2 ^= 1;
}

void checkModeButton() {
	bool readingButton1 = digitalRead(BUTTON1);
	if (readingButton1 != lastButton1State) {
        lastDebounceTime = millis();
    }
    if ((millis() - lastDebounceTime) > debounceDelay) {
        
        if ((button1State != readingButton1) && (readingButton1 == LOW)) {
        	mode++;
        	if (mode == NUM_MODES) {
        		mode = ALL_STOP;
        	}
        }
        button1State = readingButton1;
        
    }
    lastButton1State = readingButton1;
}

void loop() {
	
	//Read the stops
	XStop1 = !digitalRead(X_STOP_1);
	XStop2 = !digitalRead(X_STOP_2);
	
	checkModeButton();
    
    knob_val=analogRead(3);
    
	if (mode == RUN) {
		
		//Generate PWM value based on knob position
		pwm_x=32768+(knob_val<<2)-8192;
		
		//Read the clear the encoder registers
		delta_x=(signed short)timer_get_count(TIMER4);
		delta_y=(signed short)timer_get_count(TIMER1);
		current_x+=delta_x;
		current_y+=delta_y;
		timer_set_count(TIMER4,timer_get_count(TIMER4)-delta_x);
		timer_set_count(TIMER1,timer_get_count(TIMER1)-delta_y);
		
	} else if (mode == HOME) {
	
		pwm_x = LOW_SPEED_LEFT;
		
	} else {
		
		pwm_x = STOP;

	}
    
	//Output the PWM
	pwmWrite(PWM_PIN_X,pwm_x);
    
    printStatus();

    blinkLights();
    
    //delay(10);
}

// Force init to be called *first*, i.e. before static object allocation.
// Otherwise, statically allocated objects that need libmaple may fail.
__attribute__((constructor)) void premain() {
    init();
}

int main(void) {
    setup();

    while (true) {
        loop();
    }
    return 0;
}
