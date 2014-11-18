/*
 * TempSensor.c
 *
 * Created: 2/12/2014 11:32:55 AM
 *
 * Copyright 2014 littleBits Electronics
 *
 * This file is part of i12-temperatureSensor.
 *
 * i12-temperatureSensor is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * i12-temperatureSensor is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License at <http://www.gnu.org/licenses/> for more details.
 */ 


#include <avr/io.h>
#include <util/delay.h>
#include "USIi2c.h"

#define MCP9808 0b00011000
#define MCP9808ADDR 0b00000101
#define NCT75 0b01001000
#define SIGIN 0b0011 //mapped to PB3	

const unsigned char temperatureTable[25] = {0, 0, 0, 1, 1, 2, 2, 2, 3, 3, 4, 4, 5, 5, 5, 6, 6, 7, 7, 7, 8, 8, 9, 9, 9};

enum LastState {OFF, FARENHEIT, CELSIUS};
		
int main(void)
{

	char lastSwitchState = OFF;
	unsigned int sumbuff = 0;
	unsigned int reservebelzero = 0;
	unsigned int histbuff[16];
	unsigned char histindex = 0;
	unsigned char valout = 0;
	unsigned char minusFlag = 0;
	unsigned char plusFlag = 0;

	unsigned int ADCval = 0;
	unsigned char messageBuf[3]; //temperature returns two bytes
	unsigned char TWI_targetSlaveAddress,temp;
	long farenVal = 0;
	unsigned int i;
	
	char minimumHoldTime = 20; //The minimum time between updating the value of the temperature sensor.
	char temperatureHoldCount = 0;
	
	//Initialize the ADC for the input bitsnap signal
	ADMUX = (1 << ADLAR) | SIGIN; //set the channel mapped to the signal in. left adjust the result
	ADCSRA = 0b10000011; // Enable ADC, clock div 4
	_delay_ms(100); //let the ADC settle in
	
	ADCSRA |= (1<<6); // start an adc conversion
	while(! (ADCSRA & (1<<4))); //wait for conversion to complete
	ADCSRA &= 0b11101111; // clear the bit
	ADCval = ADCH;
	
	
	MCUCR &=0b10111111; // Make sure pull-ups are allowed.
	DDRB |= 0b00000010; //Set PB1 to output
	DDRB &= 0b11100111;
	TCCR0A = 0b00100011; //OC0B set on compare, Fast PWM mode
	TCCR0B = 0b00000001;
	TIMSK = 0b00010000;
	
	for (i = 0; i < 16; i++)
	{
		histbuff[i] = 0; //clear histbuff
	}
	
	TWI_targetSlaveAddress   = MCP9808; //MCP9808
	USI_TWI_Master_Initialise();
	
	_delay_ms(500);
	MCUCR &=0b10111111; // Make sure pull-ups are allowed.	
	DDRB &= 0b11101111;
	PORTB |= 0b00010000; //Pull-up on PB4.
	messageBuf[0] = (TWI_targetSlaveAddress<<TWI_ADR_BITS) | (FALSE<<TWI_READ_BIT);
	messageBuf[1] = MCP9808ADDR; //access the data register...
	temp = USI_TWI_Start_Read_Write( messageBuf, 2 );



	while(1)
    {
		ADCSRA |= (1<<6); // start an adc conversion
		while(! (ADCSRA & (1<<4))); //wait for conversion to complete
		ADCSRA &= 0b11101111; // clear the bit
		ADCval = ADCH;
		if (ADCval < 100)
		{
			while (ADCval < 156)
			{
				ADCSRA |= (1<<6); // start an adc conversion
				while(! (ADCSRA & (1<<4))); //wait for conversion to complete
				ADCSRA &= 0b11101111; // clear the bit
				ADCval = ADCH;
				OCR0B = 0;
			}
			lastSwitchState = OFF;
		}

		messageBuf[0] = (TWI_targetSlaveAddress<<TWI_ADR_BITS) | (TRUE<<TWI_READ_BIT); 
		_delay_ms(100);
		temp = USI_TWI_Start_Read_Write( messageBuf, 3 );
		histbuff[histindex] = ( ( (unsigned int)(messageBuf[1]&0x0F) << 8) ) + (messageBuf[2]);
		histindex = (histindex + 1) % 16;
		
		if (messageBuf[1]&0b00010000)
		{
			minusFlag++;
			plusFlag = 0;
		}
		else
		{
			minusFlag = 0;
			plusFlag++;
		}
		
		if (minusFlag > 16)
			minusFlag = 16;
		
		if (plusFlag > 16)
			plusFlag = 16;	
			
		sumbuff = 0;
		
		for (i = 0; i < 16; i++)
		{
			sumbuff += histbuff[i];
		}
		
		sumbuff >>= 4;
		
		if ( !(PINB & 0b00010000)) //If C is selected, calculate C
		{
			sumbuff >>= 4;
			if (minusFlag >= 16)
				valout = 0;
			else if (sumbuff >= 99)
				valout = 255;
			else
			{
				i = 0;
				while (sumbuff%10 > temperatureTable[i])
				{
					i++;
				}
				while (sumbuff >= 10)
				{
					i += 25;
					sumbuff -= 10;
				}
				valout = i;
				if (plusFlag < 16)
					valout = 0;
			}
			if (lastSwitchState != CELSIUS) OCR0B = valout;
			lastSwitchState = CELSIUS;
		}
		else //If F is selected, calculate F
		{
			if (minusFlag >= 16) //if temperature is below freezing (check the sign bit)
				sumbuff = 4096-sumbuff; //remove the 2s compliment
				
			farenVal = sumbuff >> 4;
			farenVal *= 1.8;
			
			if (minusFlag >= 16) //Do the conversion for a below freezing temp
				farenVal = 32 - farenVal;
			else //Do the conversion for above freezing
				farenVal += 32;
			

	
			if (farenVal > 99) farenVal = 99;
			
			if (farenVal < 0) farenVal = 0;
			sumbuff = (unsigned int)farenVal;
			
			if ( (plusFlag < 16) && (minusFlag < 16) ) sumbuff = 32;
			
			i = 0;
			while (sumbuff%10 > temperatureTable[i])
			{
				i++;
			}
			while (sumbuff >= 10)
			{
				i += 25;
				sumbuff -= 10;
			}
			valout = i;
			if (lastSwitchState != FARENHEIT) OCR0B = valout;
			lastSwitchState = FARENHEIT;
		}

		temperatureHoldCount++;
		if (temperatureHoldCount == 20)
		{
			OCR0B = valout;
		}
    }
}