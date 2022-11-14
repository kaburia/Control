/*
 * GccApplication5.c
 *
 * Created: 14/11/2022 10:36:34
 * Author : Austin
 */ 

#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>



unsigned char password[4] = {0b00000010, 0b00001000, 0b00000001, 0b00000100};
unsigned char data[4] = {};
int approve[4] = {};


void check_password();

void led();

int main(void)
{
	
	DDRB = 0b00000000;
	DDRC = 0b00011111;
	DDRD = 0b00000011;
	PORTB = 0b00001111; // Setting PB0, PB1,PB2,PB3
	//int count =0;
	
    /* Replace with your application code */
    while (1) 
    {
		
		if (PINB != 0b00001111){
			led();
		}
		if (approve[0]==4 && approve[1]==5 && approve[2]==6 && approve[3]==7)// Checking if all buttons were pressed
		{
		check_password();
		memset(approve, 0, sizeof(approve));	// Releasing the values set to the check buttons
		}
		
		
    }
	
	
	return 0;
}


void check_password(){
	if (data[0]==password[0] && data[1]==password[1] && data[2]==password[2] && data[3]==password[3])
	{
			PORTD = PORTD | (1<<PD0);
			_delay_ms(500);
			PORTD = 0b00000000;
			_delay_ms(500);
			
			PORTC = 0b00000000;// Turning off the LEDs
			
			PORTD = 0b00000001;
			_delay_ms(500);
			PORTD = 0b00000000;
			_delay_ms(500);
			
			PORTD = 0b00000001;
			_delay_ms(500);
			PORTD = 0b00000000;
			_delay_ms(500);
			
			PORTD = 0b00000001;
			_delay_ms(500);
			PORTD = 0b00000000;
			_delay_ms(500);
		}
		
		
		else
		{
			PORTD = PORTD | (1<<PD1);
			_delay_ms(500);
			PORTD = 0b00000000;
			_delay_ms(500);
			
			PORTC = 0b00000000;// Turning off the LEDs
			
			PORTD = PORTD | (1<<PD1);
			_delay_ms(500);
			PORTD = 0b00000000;
			_delay_ms(500);
			
			PORTD = PORTD | (1<<PD1);
			_delay_ms(500);
			PORTD = 0b00000000;
			_delay_ms(500);
			
			PORTD = PORTD | (1<<PD1);
			_delay_ms(500);
			PORTD = 0b00000000;
			_delay_ms(500);
		}
		
	
	
}

void red_led_blink(){
	
}

void led(){
	int i = 0;
	while (i <4){
		if (PINB == 0b00001110){
			PORTC = PORTC | (1<<PC0);
			approve[0] = 4;
			if (i==2)
			{
				data[2] = 0b00000001;
				i ++;
			}
			else
			{
				data[i] = 0b00000001;
			}
		}
		else if (PINB == 0b00001101){
			PORTC = PORTC | (1<<PC1);
			approve[1] = 5;
			if (i==0)
			{
				data[0] = 0b00000010;
				i ++;
			}
			else
			{
				data[i] = 0b00000010;
			}
		}
		else if (PINB == 0b00001011){
			PORTC = PORTC | (1<<PC2);
			approve[2] = 6;
			if (i==3)
			{
				data[3] = 0b00000100;
				i ++;
			}
			else
			{
				data[i] = 0b00000100;
			}
		}
		else if (PINB == 0b00000111){
			PORTC = PORTC | (1<<PC3);
			approve[3] = 7;
			if (i==1)
			{
				data[1] = 0b00001000;
				i ++;
			}
			else
			{
				data[i] = 0b00001000;
			}
		}
		
	}
}