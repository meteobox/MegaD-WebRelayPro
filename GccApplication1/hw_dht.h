/*
DHT Library 0x03

copyright (c) Davide Gironi, 2012

Released under GPLv3.
Please refer to LICENSE file for licensing information.

References:
  - DHT-11 Library, by Charalampos Andrianakis on 18/12/11
*/


#ifndef DHT_H_
#define DHT_H_

#include <stdio.h>
#include <avr/io.h>

//timeout retries
#define DHT_TIMEOUT 200
extern uint8_t port_num;
extern char port_letter;
extern uint8_t dht_type;

extern volatile uint8_t *w_port;
extern volatile uint8_t *w_ddr;
extern volatile uint8_t *w_pin;


extern int8_t dht_gettemperature(int8_t *temperature);
extern int8_t dht_gethumidity(int8_t *humidity);
extern int8_t dht_gettemperaturehumidity(int16_t *temperature, int16_t *humidity, int8_t port_m);

extern volatile uint8_t isr_flag;

#endif
