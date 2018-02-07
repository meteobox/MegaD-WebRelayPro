/*
DHT Library 0x03

copyright (c) Davide Gironi, 2012

Released under GPLv3.
Please refer to LICENSE file for licensing information.
*/

#include <stdio.h>
#include <string.h>
#include <avr/io.h>
#include "timeout.h"
#include "hw_dht.h"

int8_t dht_getdata(int16_t *temperature, int16_t *humidity, int8_t port_m) {
	uint8_t bits[5];
	uint8_t i,j = 0;

	memset(bits, 0, sizeof(bits));

	/*
	if ( port_letter == 'D' )
	{
		DDRD |= (1<<port_num); //output
		PORTD |= (1<<port_num); //high
	}
	else if ( port_letter == 'B' )
	{
		DDRB |= (1<<port_num); //output
		PORTB |= (1<<port_num); //high
	}
	else if ( port_letter == 'C' )
	{
		DDRC |= (1<<port_num); //output
		PORTC |= (1<<port_num); //high
	}
	*/

	*w_ddr |= (1<<port_num); //output
	*w_port |= (1<<port_num); //high

	_delay_ms(100);

	//send request
	/*
	if ( port_letter == 'D' )
	PORTD &= ~(1<<port_num); //low
	else if ( port_letter == 'B' )
	PORTB &= ~(1<<port_num); //low
	else if ( port_letter == 'C' )
	PORTC &= ~(1<<port_num); //low
	*/

	*w_port &= ~(1<<port_num); //low

	//if ( port_m == 2 )
	_delay_us(500);
	//else
	//_delay_ms(18);

	isr_flag = 0;

	while (isr_flag == 0)
	_delay_us(1);

	/*
	if ( port_letter == 'D' )
	{
		PORTD |= (1<<port_num); //high
		DDRD &= ~(1<<port_num); //input
	}
	else if ( port_letter == 'B' )
	{
		PORTB |= (1<<port_num); //high
		DDRB &= ~(1<<port_num); //input
	}
	else if ( port_letter == 'C' )
	{
		PORTC |= (1<<port_num); //high
		DDRC &= ~(1<<port_num); //input
	}
	*/

	*w_port |= (1<<port_num); //high
	*w_ddr &= ~(1<<port_num); //input

	_delay_us(40);

	//check start condition 1
	/*
	if ( port_letter == 'D' )
	{
		if((PIND & (1<<port_letter))) {
			return -1;
		}
	}
	else if ( port_letter == 'B' )
	{
		if((PINB & (1<<port_letter))) {
			return -1;
		}
	}
	else if ( port_letter == 'C' )
	{
		if((PINC & (1<<port_letter))) {
			return -1;
		}
	}
	*/
	if((*w_pin & (1<<port_letter))) {
		return -1;
	}

	_delay_us(80);

	//check start condition 2
	/*
	if ( port_letter == 'D' )
	{
		if(!(PIND & (1<<port_num))) {
			return -1;
		}
	}
	else if ( port_letter == 'B' )
	{
		if(!(PINB & (1<<port_num))) {
			return -1;
		}
	}
	else if ( port_letter == 'C' )
	{
		if(!(PINC & (1<<port_num))) {
			return -1;
		}
	}
	*/
	if(!(*w_pin & (1<<port_num))) {
		return -1;
	}

	
	_delay_us(80);

	//read the data
	uint16_t timeoutcounter = 0;
	for (j=0; j<5; j++) { //read 5 byte
		uint8_t result=0;
		for(i=0; i<8; i++) {//read every bit
			timeoutcounter = 0;
			
			/*
			if ( port_letter == 'D' )
			{
				while(!(PIND & (1<<port_num))) { //wait for an high input (non blocking)
					timeoutcounter++;
					if(timeoutcounter > DHT_TIMEOUT) {
						return -1; //timeout
					}
				}
			}
			else if ( port_letter == 'B' )
			{
				while(!(PINB & (1<<port_num))) { //wait for an high input (non blocking)
					timeoutcounter++;
					if(timeoutcounter > DHT_TIMEOUT) {
						return -1; //timeout
					}
				}
			}
			else if ( port_letter == 'C' )
			{
				while(!(PINC & (1<<port_num))) { //wait for an high input (non blocking)
					timeoutcounter++;
					if(timeoutcounter > DHT_TIMEOUT) {
						return -1; //timeout
					}
				}
			}
			*/

			while(!(*w_pin & (1<<port_num))) { //wait for an high input (non blocking)
				timeoutcounter++;
				if(timeoutcounter > DHT_TIMEOUT) {
					return -1; //timeout
				}
			}


			_delay_us(30);

			/*
			if ( port_letter == 'D' )
			{
				if(PIND & (1<<port_num)) //if input is high after 30 us, get result
					result |= (1<<(7-i));
			}
			else if ( port_letter == 'B' )
			{
				if(PINB & (1<<port_num)) //if input is high after 30 us, get result
					result |= (1<<(7-i));
			}
			else if ( port_letter == 'C' )
			{
				if(PINC & (1<<port_num)) //if input is high after 30 us, get result
					result |= (1<<(7-i));
			}
			*/

			if(*w_pin & (1<<port_num)) //if input is high after 30 us, get result
				result |= (1<<(7-i));

			timeoutcounter = 0;

			/*
			if ( port_letter == 'D' )
			{
				while(PIND & (1<<port_num)) { //wait until input get low (non blocking)
					timeoutcounter++;
					if(timeoutcounter > DHT_TIMEOUT) {
						return -1; //timeout
					}
				}
			}
			else if ( port_letter == 'B' )
			{
				while(PINB & (1<<port_num)) { //wait until input get low (non blocking)
					timeoutcounter++;
					if(timeoutcounter > DHT_TIMEOUT) {
						return -1; //timeout
					}
				}
			}
			else if ( port_letter == 'C' )
			{
				while(PINC & (1<<port_num)) { //wait until input get low (non blocking)
					timeoutcounter++;
					if(timeoutcounter > DHT_TIMEOUT) {
						return -1; //timeout
					}
				}
			}
			*/

			while(*w_pin & (1<<port_num)) { //wait until input get low (non blocking)
				timeoutcounter++;
				if(timeoutcounter > DHT_TIMEOUT) {
					return -1; //timeout
				}
			}


		}
		bits[j] = result;
	}

	//reset port

	/*
	if ( port_letter == 'D' )
	{
		DDRD &= ~(1<<port_num); //output
		PORTD |= (1<<port_num); //low
	}
	else if ( port_letter == 'B' )
	{
		DDRB &= ~(1<<port_num); //output
		PORTB |= (1<<port_num); //low
	}
	else if ( port_letter == 'C' )
	{
		DDRC &= ~(1<<port_num); //output
		PORTC |= (1<<port_num); //low
	}
	*/

	*w_ddr &= ~(1<<port_num); //output
	*w_port |= (1<<port_num); //low

	//_delay_ms(100);

	//check checksum
	if ((uint8_t)(bits[0] + bits[1] + bits[2] + bits[3]) == bits[4]) {
		//if ( port_m == 2 )
		{
			uint16_t rawhumidity = bits[0]<<8 | bits[1];
			uint16_t rawtemperature = bits[2]<<8 | bits[3];
			if(rawtemperature & 0x8000) {
				//*temperature = (float)((rawtemperature & 0x7FFF) / 10.0) * -1.0;
				//*temperature = ((rawtemperature & 0x7FFF) / 10) * -1;
				*temperature = ((rawtemperature & 0x7FFF) * 10) * -1;
			} else {
				//*temperature = (float)(rawtemperature)/10.0;
				//*temperature = (rawtemperature)/10;
				*temperature = rawtemperature * 10;
			}
			//*humidity = (float)(rawhumidity)/10.0;
			//*humidity = (rawhumidity)/10;
			*humidity = rawhumidity * 10;
		}
		/*
		else
		{
			*temperature = bits[2] * 100;
			*humidity = bits[0] * 100;
		}
		*/
		//#endif
		return 0;
	}

	return -1;
}

int8_t dht_gettemperaturehumidity(int16_t *temperature, int16_t *humidity, int8_t port_m) {
return dht_getdata(temperature, humidity, port_m);
}
