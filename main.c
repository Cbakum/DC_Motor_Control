#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include "i2c.h"
#include "SSD1306.h"
#include <util/delay.h>

#define DC_MOTOR PE2
#define UP_COUNT PB5
#define DOWN_COUNT PB7
#define SENSOR_PIN PD2
#define SPEED_CALC_FLAG 1
#define SENSOR_WHEEL_OPENINGS 2

//global variables for interrupts
volatile int overflow_counter = 0;
volatile int sensor_counter = 0;


ISR(TIMER0_COMPA_vect){ //Timer0 Interrupt
	overflow_counter++; //increase counter variable
}
ISR(INT0_vect, ISR_BLOCK){ //each time sensor goes low
	sensor_counter++; 
}
ISR(TIMER1_COMPA_vect, ISR_BLOCK){
	PORTE &= ~(1<<DC_MOTOR); //turn PortE2 low
}
ISR(TIMER1_COMPB_vect, ISR_BLOCK){ 
	PORTE |= (1<<DC_MOTOR); //turn PortE2 high
}

void OLED_display(int *countInt, char *dutyCycle, int *rpm){ //update OLED display
	OLED_SetCursor(0, 0);
	OLED_Printf("Colin Bakum");
	OLED_SetCursor(1, 0);
	OLED_Printf("Motor Control:");
	OLED_SetCursor(3, 0);
	OLED_Printf("Mode:        ");
	OLED_DisplayNumber(C_DECIMAL_U8,*countInt,2);
	OLED_SetCursor(4, 0);
	OLED_Printf("Duty Cycle:  ");
	OLED_DisplayNumber(C_DECIMAL_U8,*dutyCycle,3);
	OLED_SetCursor(5, 0);
	OLED_Printf("RPM:         ");
	OLED_DisplayNumber(C_DECIMAL_U8,*rpm,5);
	}
		
void checkJoystick(int *countInt, char *lastInput, char *dutyCycle){
	char joystick = (0b11110000&PINB); //mask top four bits
	if(*lastInput != joystick){ //check for change
		if(joystick&(1<<UP_COUNT)){
			if(*countInt < 10) 
			*countInt++; // increment if not 10
		}
		if(joystick&(1<<DOWN_COUNT)){
			if(*countInt > 0) 
			*countInt--; // decrement if not 0
		}
		*dutyCycle = *countInt*10;//update dutyCycle variable for OLED display
		*lastInput = joystick;
	}
}

int rpm_calc(){
	int rpm = ((sensor_counter/((256*0.0625*0.000001*1024)*200))*60)/SENSOR_WHEEL_OPENINGS; //calculate RPM
	sensor_counter = 0; 
	overflow_counter = 0; //reset counter values
	return rpm;
}

int main(void){

	volatile unsigned char duty_cycle = 0;
	volatile unsigned char last_input = 0;
	volatile int count_int = 0;
	volatile int RPM = 0;
	const double dc_adj=0.1; //to turn counter into a percentage
	const int n=1; //speed controller. set 1 as speed reduction not needed
	
	OLED_Init();
	_delay_ms(5);
	OLED_Clear();

	//initialize ports
	DDRB = 0x00;
	PORTB = 0xA0;
	DDRE |= (1<<DC_MOTOR);//set motor pin to output
	DDRD &= ~(1<<SENSOR_PIN)); 
	
	// initialize timer1
	OCR1A=0xFFFF; //to be altered during the while loop for PWM
	OCR1B=0XFFFF; //constant
	TCNT1H = 0x00;
	TCNT1L = 0x00;
	TCCR1A |= (1<<COM1A0)|(1<<COM1B0);//compare A and B for interrupts
	TCCR1B |= (1<<CS10); //initialize w/ no prescaler
	
	TCCR0A |= (1<<WGM01);//CTC mode
	TCCR0B |= (1<<CS00)|(1<<CS02); //1024 prescaler
	TIMSK0 |= (1<<OCIE0A); //unmask interrupt on compare
	OCR0A = 0xff;

	//enable INT0 interrupt
	EICRA |= (1<<ISC01); //falling edge interrupt
	EIMSK |= (1<<INT0);
	
	sei();
	
	while(1){
		if (count_int == 0) TIMSK1 &= ~(1<<OCIE1B); //umask interrupt that turns pulse high
		else if (count_int == 10) TIMSK1 &= ~(1<<OCIE1A); //unmask interrupt that turns pulse low
		else TIMSK1 |= (1<<OCIE1A)|(1<<OCIE1B); //mask interrupts for PWM

		if (overflow_counter >= 200){
		  RPM = rpm_calc();
		}
		
		OCR1A = 0xFFFF * dc_adj * count_int/n; //
		checkJoystick(&count_int, &last_input, &duty_cycle);
		OLED_display(&count_int, &duty_cycle, &RPM);
	}
}
