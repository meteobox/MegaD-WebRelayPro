/********************************************
 * Author: Andrey_B (Home Automation http://www.ab-log.ru), Guido Socher (TCP/IP stack)
 * Copyright: GPL V2
 * See http://www.gnu.org/licenses/gpl.html
 *
 * Home Automation universal Ethernet module
 * Remote Control and Configuration
 * Port: IN/OUT/PWM/ADC support
 * HTTP interface 
 *
 * Chip type: Atmega328 with ENC28J60
 *
 * ВНИМАНИЕ! Перед загрузкой прошивки убедитесь,
 * что последовательность портов, описанная в массиве "aio"
 * соответствует вашей версии устройства.
 * 
 *********************************************/
/*
Project sources url: https://github.com/meteobox/MegaD-WebRelayPro

WebMeteoBox Mod by http://webmeteobox.ru
07.02.2018 Added UDP search and configure device ip,gateway
WebRelayPro device support http://webmeteobox.ru/webrelaypro.html


GET /md.php?mdid=12345&all=OFF;OFF;OFF;OFF;OFF;OFF;OFF;ON;ON;ON;OFF;OFF;OFF;OFF;282;276 HTTP/1.1
*/

#include <avr/io.h>
#include <avr/wdt.h>
#include <string.h>
#include <stdlib.h>
#include "websrv_help_functions.h"
#include "ip_arp_udp_tcp.h"
#include "enc28j60.h"
#include "timeout.h"
#include "tcp_state.h"
#include "net.h"
#include <stdio.h>
#include <avr/eeprom.h> 
#include <avr/interrupt.h>
// For DHT11/DHT22 humidity sensor
#include "hw_dht.h"

// Firmware version
//#define MEGADFWV "3.38b7"

#define ENABLE_UDP_SEARCH
#define CONFIG_CLIENT_PORT			1444
#define CONFIG_SERVER_PORT			8444


// Default MAC address
uint8_t mymac[6] = {0x00,0x15,0xc1,0xa8,0x00,0x0e};
// Default IP address
uint8_t myip[4] = {192,168,1,144};

// Default Netmask
static uint8_t netmask[4] = {255,255,255,0};
// Default HTTP port
#define HTTPPORT 80
// MAC resolving defenition
#define TRANS_NUM_GWMAC 1
static uint8_t gwmac[6] = {255,}; //&&&
#define TRANS_NUM_WEBMAC 2
//static uint8_t www_gwmac[6];
static uint8_t www_gwmac2[6];

// EEPROM 
// EEMEM IP address
#define ee_ip_addr	0x03F6
// FW Update flag
#define ee_fw_up	0x03F5

// IP address
uint8_t _ip_addr[4];
// EEMEM server IP address
uint8_t EEMEM ee_sip_addr[4];
// EEMEM Gateway IP address
uint8_t EEMEM ee_gw_addr[4];
// server IP address
uint8_t _sip_addr[4];
// EEMEM server script (MAX 14 bytes!)
uint8_t EEMEM ee_spt[15];
// EEMEM password (MAX 5 bytes!)
uint8_t EEMEM ee_pwd[6];
// server script
char _spt[16];
// password
static char password[7];
// config preset
uint8_t EEMEM ee_preset;
static uint8_t _preset;
// alarm temp
uint8_t EEMEM ee_alarm_temp;
static uint8_t _alarm_temp;

// Temp check flag /*Temp-check*/
//uint8_t EEMEM ee_temp_check;
//uint8_t temp_check_flag = 0;

// EEMEM ab-srv ID (MAX 5 bytes!)
uint8_t EEMEM ee_mdid[5];
char _mdid[6];

// Available IO
#define IO_SIZE 16
// MegaD-328-7I7O Ver 4-5 sequence             



//                     [  R   E  L  A  Y  ]    [1-wire]    SDA   SCL   A2    A3   [ref]  D8    RX    TX   
//                     P0    P1    P3    P3    P4    P5    P6    P7    P8    P9    P10   P11   P12   P13   A6    A7
char *aio[IO_SIZE] = {"D2", "D3", "D4", "D5", "D6", "D7", "C4", "C5", "C2", "C3", "C1", "B0", "D0", "D1", "A6", "A7"};
	
	
	
//char *aio[IO_SIZE] = {"C5", "C4", "C3", "C2", "C1", "C0", "D7", "B1", "D0", "D1", "D3", "D4", "D5", "D6", "A6", "A7"};	
// MegaD-328-7I7O Ver 1-3 sequence
//char *aio[IO_SIZE] = {"C5", "D0", "C4", "D1", "C3", "C2", "D3", "B1", "D7", "D6", "D5", "C0", "D4", "C1", "A6", "A7"};

// Action
uint8_t EEMEM ee_cmd[IO_SIZE][23];
// NetAction
uint8_t EEMEM ee_eth_cmd[IO_SIZE][17];
// Temp check flag
uint8_t EEMEM ee_netact_flag[IO_SIZE];

// Initializing buffer for HTTP
#define BUFFER_SIZE 801
static uint8_t buf[BUFFER_SIZE+1];
// Buffer for URL parameters
static char gStrbuf[25];

// EEPROM
//Port type
uint8_t EEMEM ee_port_type[IO_SIZE];
static uint8_t _port_type[IO_SIZE];

//Port default state
uint8_t EEMEM ee_port_d[IO_SIZE];
static uint8_t _port_d[IO_SIZE];

//Port default mode (Switch/PWM)
uint8_t EEMEM ee_port_m[IO_SIZE];
static uint8_t _port_m[IO_SIZE];

//Port misc param 
// В данный момент используется для АЦП, для флага поведения входа
int16_t EEMEM ee_port_misc[IO_SIZE];
static int16_t _port_misc[IO_SIZE];
static int16_t port_misc_cur[IO_SIZE];

uint16_t EEMEM ee_srv_port;
uint16_t _srv_port;

// Циклическая работа с сервером
uint8_t EEMEM ee_srv_loop;
uint8_t srv_loop_timeout = 0;
uint32_t srv_loop_timer = 0;

#define SRV_LOOP_TIMEOUT 12000 // 12000 * 5ms = 60 sec

// Port act status array
static int8_t port_act[IO_SIZE];

// Гистерезис
uint16_t EEMEM ee_port_hyst[IO_SIZE];

//Port input counter + hyst
uint16_t port_cnt[IO_SIZE];

// Service
char temp[42];
char shost[20];
uint8_t i;
// Reset flag
uint8_t reset_flag = 0;
// ADC/DS18B20 check flag
uint8_t adc_check_flag = 0;
uint8_t cur_input;
//static uint8_t cur_input2[IO_SIZE];
// Управление таймаутами ответа сервера по каждому порту
static uint16_t srv_timeout[IO_SIZE];
static uint16_t srv_timeout_act[IO_SIZE];

static uint8_t cmd_delay_p[IO_SIZE];

// ARP таймаут
static uint16_t arp_timeout = 0;
// If is_set gateway IP
//static uint8_t gw_set = 0;
// Невозможно открыть две TCP-сессии. Флаги для управления очередностью сессий
// В этом флаге сохраняется номер сработавшего входа
//static int8_t send_eth_flag = -1;
// Этот флаг устанавливается, когда завершена TCP-сессия с основным сервером
static int8_t send_eth_flag2 = -1;

// Debouncing
char input_state;      //Debounced and inverted key state:
char input_press;      //Key Press Detected  
char input_release;      //Key Release Detected  
char input_state2;      //Debounced and inverted key state:
char input_press2;      //Key Press Detected  
char input_release2;      //Key Release Detected  
char input_state3;      //Debounced and inverted key state:
char input_press3;      //Key Press Detected  
char input_release3;      //Key Release Detected  

// DHT vars
int16_t dht11_temperature = 0;
int16_t dht11_humidity = 0;
uint8_t volatile isr_flag=0;

uint8_t port_num = 0;
char port_letter;
uint8_t dht_type;

char const selected[] PROGMEM = " selected";

// function for software reset of AVR
#define RESET() (((void(*)(void))(char *)0x0000)()) 
/*
// set output to VCC, red LED off
#define LEDON PORTB|=(1<<PORTB0)
// set output to GND, red LED on
#define LEDOFF PORTB&=~(1<<PORTB0)
// to test the state of the LED
#define LEDISOFF PORTB&(1<<PORTB0)
*/

// set output to VCC, red LED off
#define LEDON 
// set output to GND, red LED on
#define LEDOFF 
// to test the state of the LED
#define LEDISOFF 

uint32_t adc_timer = 840;
//uint32_t temp_timer = 0;

// Оптимизация использования Flash-памяти
const PROGMEM char html_form[] = "<form action=/";
const PROGMEM char html_submit[] = "<br><input type=submit value=Save></form>";
const PROGMEM char html_input_hidden[] = "<input type=hidden name=";
const PROGMEM char html_checked[] = " checked";
const PROGMEM char html_cl[] = ">";
const PROGMEM char html_checked_cl[] = " checked>";
const PROGMEM char html_ahref[] = "<a href=/";
const PROGMEM char html_br[] = "<br>";
const PROGMEM char html_cbr[] = "><br>";
const PROGMEM char html_scbr[] = "\"><br>";
const PROGMEM char html_out[] = "OUT";
const PROGMEM char html_in[] = "IN";
const PROGMEM char html_on[] = "ON";
const PROGMEM char html_off[] = "OFF";
const PROGMEM char html_pt[] = "/?pt=";
const PROGMEM char html_cmd[] = "&cmd=";

//const PROGMEM char input_name[] = "<input name=";
//const PROGMEM char html_back[] = ">Back</a>";

typedef void (*AppPtr_t) (void);
AppPtr_t Bootloader = (AppPtr_t)0x03800;

volatile uint8_t *w_port;
volatile uint8_t *w_ddr;
volatile uint8_t *w_pin;
volatile uint8_t *w_ocr;

//###########################################

//setup connection
//#define DS18B20_PORT PORTC
//#define DS18B20_DDR DDRC
//#define DS18B20_PIN PINC
//#define DS18B20_DQ 5

//commands
#define DS18B20_CMD_SKIPROM 0xcc
#define DS18B20_CMD_CONVERTTEMP 0x44
#define DS18B20_CMD_RSCRATCHPAD 0xbe
#define DS18B20_CMD_WSCRATCHPAD 0x4e
#define DS18B20_CMD_READROM 0x33
/*
#define DS18B20_CMD_CPYSCRATCHPAD 0x48
#define DS18B20_CMD_RECEEPROM 0xb8
#define DS18B20_CMD_RPWRSUPPLY 0xb4
#define DS18B20_CMD_SEARCHROM 0xf0
#define DS18B20_CMD_READROM 0x33
#define DS18B20_CMD_MATCHROM 0x55
#define DS18B20_CMD_ALARMSEARCH 0xec
*/

//decimal conversion table
#define DS18B20_DECIMALSTEPS_9BIT  5000 //0.5
#define DS18B20_DECIMALSTEPS_10BIT 2500 //0.25
#define DS18B20_DECIMALSTEPS_11BIT 1250 //0.125
#define DS18B20_DECIMALSTEPS_12BIT 625  //0.0625
#define DS18B20_DECIMALSTEPS DS18B20_DECIMALSTEPS_12BIT


//uint8_t ds18b20_reset(int8_t portnum, volatile uint8_t *w_port, volatile uint8_t *w_ddr, volatile uint8_t *w_pin) {
uint8_t ds18b20_reset(int8_t portnum) {
	uint8_t i;
	cli();
	//low for 480us
	//DS18B20_PORT &= ~ (1<<portnum); //low
	//DS18B20_DDR |= (1<<portnum); //output

	*w_port &= ~ (1<<portnum); //low
	*w_ddr |= (1<<portnum); //output

	_delay_us(480);

	//release line and wait for 60uS
	//DS18B20_DDR &= ~(1<<portnum); //input
	*w_ddr &= ~(1<<portnum); //input
	_delay_us(70);

	//get value and wait 420us
	//i = (DS18B20_PIN & (1<<portnum));
	i = (*w_pin & (1<<portnum));
	_delay_us(410);
	sei();
	//return the read value, 0=ok, 1=error
	return i;
}

/*
 * write one bit
 */
void ds18b20_writebit(uint8_t bit, int8_t portnum){
	cli();
	//low for 1uS
	//DS18B20_PORT &= ~ (1<<portnum); //low
	//DS18B20_DDR |= (1<<portnum); //output
	
	*w_port &= ~ (1<<portnum); //low
	*w_ddr |= (1<<portnum); //output

	_delay_us(10); //1

	//if we want to write 1, release the line (if not will keep low)
	if(bit)
	//DS18B20_DDR &= ~(1<<portnum); //input
	*w_ddr &= ~(1<<portnum); //input

	//wait 60uS and release the line
	_delay_us(55); //60
	//DS18B20_DDR &= ~(1<<portnum); //input
	*w_ddr &= ~(1<<portnum); //input
	sei();
}       

/*
 * read one bit
 */
uint8_t ds18b20_readbit(uint8_t portnum){
	uint8_t bit=0;
	cli();

	//low for 1uS
	*w_port &= ~ (1<<portnum); //low
	*w_ddr |= (1<<portnum); //output
	_delay_us(1); //1

	//release line and wait for 14uS
	*w_ddr &= ~(1<<portnum); //input
	_delay_us(14); //14

	//read the value
	if(*w_pin & (1<<portnum))
		bit=1;

	//wait 45uS and return read value
	_delay_us(45); //45
	sei();
	return bit;
}

/*
 * write one byte
 */
void ds18b20_writebyte(uint8_t byte, uint8_t portnum){
	uint8_t i=8;
	while(i--){
		ds18b20_writebit(byte&1, portnum);
		byte >>= 1;
	}
}

/*
 * read one byte
 */
uint8_t ds18b20_readbyte(uint8_t portnum){
	uint8_t i=8, n=0;
	while(i--){
		n >>= 1;
		n |= (ds18b20_readbit(portnum)<<7);
	}
	return n;
}

/*
 * get temperature
 */
int ds18b20_gettemp(int8_t portnum) {
	uint8_t temperature[2];
	int8_t digit;
	//uint16_t decimal;
	int16_t decimal;

	if ( ds18b20_reset(portnum) == 0) //reset
	{
		ds18b20_writebyte(DS18B20_CMD_SKIPROM, portnum); //skip ROM
		ds18b20_writebyte(DS18B20_CMD_RSCRATCHPAD, portnum); //read scratchpad

		//read 2 byte from scratchpad
		temperature[0] = ds18b20_readbyte(portnum);
		temperature[1] = ds18b20_readbyte(portnum);

		ds18b20_reset(portnum); //reset

		ds18b20_writebyte(DS18B20_CMD_SKIPROM, portnum); //skip ROM
		ds18b20_writebyte(DS18B20_CMD_CONVERTTEMP, portnum); //start temperature conversion
		*w_ddr |= (1<<portnum); //output
		*w_port |= (1<<portnum);

		//store temperature integer digits
		digit = temperature[0]>>4;
		digit |= (temperature[1]&0x7)<<4;
	
		//store temperature decimal digits
		decimal = temperature[0]&0xf;
		decimal *= DS18B20_DECIMALSTEPS;

		if ( temperature [1] >= 0x80 )
		{
			digit = digit - 127;
			decimal = (10000 - decimal) *-1;
		}

		return digit * 100 + decimal / 100;
	}
	else
	return 13000;
}

uint8_t crc8(uint8_t crc, uint8_t inbyte)
{
	uint8_t i;
	for (i = 8; i; i--)
	{
		uint8_t mix = (crc ^ inbyte) & 0x01;
		crc >>= 1;
		if (mix) crc ^= 0x8C;
		inbyte >>= 1;
	}
	return crc;
}

//###########################################
/*
int16_t readchiptemp(void)
{
	ADMUX = (3 << REFS0) | (8 << MUX0); // 1.1V REF, channel#8 is temperature
	ADCSRA |= (1 << ADEN) | (6 << ADPS0);       // enable the ADC div64
	_delay_ms(20);
	ADCSRA |= (1 << ADSC);      // Start the ADC

	while (ADCSRA & (1 << ADSC));       // Detect end-of-conversion
	// The offset of 324.31 could be wrong. It is just an indication.
	// The returned temperature is in degrees Celcius.
	//return (ADCW * 100 - 32431) / 122;
	int16_t t;
	t = ADCW * 100 - 32431;
	t = t / 122;
	return t;
} 
*/

uint16_t http200ok(void)
{
	//return(fill_tcp_data_p(buf,0,PSTR("HTTP/1.0 200 OK\r\nContent-Type: text/html\r\nPragma: no-cache\r\n\r\n")));
	return(fill_tcp_data_p(buf,0,PSTR("HTTP/1.0 200 OK\r\nContent-Type: text/html\r\n\r\n")));
}

int8_t analyse_get_url(char *str)
{
	uint8_t loop=0;
	// the first slash:
	if ( *str == '/' )
	str++;
	else
	return(-1);

        // move forward to the first space or '/'
	while(loop < 6)
	{
		if( *str==' ' || *str=='/' )
		{
			break;
                }

		str++;
		loop++;
	}

	str = str - loop;

	if ( strlen(password) == loop && strncmp(password,str,loop) == 0 )
	return(1);

	return(-1);

}

static uint8_t decode_ip(char *in,uint8_t *out)
{
	strncpy(temp,in,sizeof(temp));
	char *dig;
	dig=strtok(temp,".");

	for(i=0 ; i<4 && dig ;i++,dig=strtok(NULL,"."))
	out[i]=(uint8_t)strtoul(dig,NULL,10);

	return i;
}

void sw_ocr (uint8_t port_num)
{
	if ( port_num == 6 )
	w_ocr = &OCR0A;
	else if ( port_num == 5 )
	w_ocr = &OCR0B;
	//else if ( port_num == 3 )
	else
	w_ocr = &OCR2B;
}

void sw_port (char port_letter)
{
	if ( port_letter == 'D' )
	{
		w_port = &PORTD;
		w_ddr = &DDRD;
		w_pin = &PIND;
	}
	else if ( port_letter == 'C' )
	{
		w_port = &PORTC;
		w_ddr = &DDRC;
		w_pin = &PINC;
	}
	else if ( port_letter == 'B' )
	{
		w_port = &PORTB;
		w_ddr = &DDRB;
		w_pin = &PINB;
	}
}

// Функция, которая осуществляет парсинг и выполнение команд
void port_execute(char *srv_cmd, uint8_t c_input, uint8_t cmd_shift, uint8_t port_state)
{
	char p_num[3] = "FF";
	char pwm[4];
	uint8_t cnt=0;
	uint8_t p_flag=0;
	char port_letter;
	int8_t port_num;

	char cmd_delay[6];

	for ( i = 0; i < strlen(srv_cmd); i++ )
	{
		if ( srv_cmd[i] == '\n' )
		{
			p_flag = 0;
			p_num[0] = 'F';
			p_num[1] = 'F';
			pwm[0] = '\0';
			cnt = 0;
		}
		else if ( srv_cmd[i] == ':' && p_flag != 3 )
		p_flag = 1;
		else if ( srv_cmd[i] == ';' )
		p_flag = 0;
		else if ( srv_cmd[i] == 'p' )
		p_flag = 2;
		else if ( srv_cmd[i] == 'a' )
		p_flag = 3;
		else
		{
			// Управление диммированием входами
			if ( srv_cmd[i] == '+' || srv_cmd[i] == '~')
			p_flag = 5;
			else if ( srv_cmd[i] == '-' )
			p_flag = 6;

			if ( p_flag == 0 )
			{
				if ( p_num[0] != 'F' )
				p_num[1] = srv_cmd[i];
				else
				p_num[0] = srv_cmd[i];
			}
			else if ( p_flag == 1 || p_flag == 4 || p_flag >= 5 ) // ***
			{
				// Сбрасываем флаг/счетчик длинного нажатия для Action, отличных от управления PWM (+/-/~)
				//if ( p_flag < 5 && c_input != 100 )
				if ( p_flag < 5 && c_input != 100 && _port_type[c_input] == 0) // UPDATE*
				port_misc_cur[c_input] = 0;

				if ( p_num[1] == 'F' )
				p_num[1] = '\0';

				int8_t n = atoi(p_num);
				port_letter = aio[n][0];
				port_num = atoi(&aio[n][1]);

				if ( srv_cmd[i] == '~' )
				{
					// Задействуем переменную srv_timeout для экономии памяти, так как для OUT эта переменная не используется
					// Ну а что делать... Кому сейчас легко?
					if ( srv_timeout[n] == 2 )
					p_flag = 6;
					else
					p_flag = 5;
				}

				// ОПТИМИЗАЦИЯ!
				/*
				if ( port_num == 6 )
				w_ocr = &OCR0A;
				else if ( port_num == 5 )
				w_ocr = &OCR0B;
				//else if ( port_num == 3 )
				else
				w_ocr = &OCR2B;
				*/
				sw_ocr(port_num);
				sw_port(port_letter);

				// *****************************
				/*
				if ( port_letter == 'D' )
				w_port = &PORTD;
				else if ( port_letter == 'C' )
				w_port = &PORTC;
				else if ( port_letter == 'B' )
				w_port = &PORTB;
				*/
				// *****************************


				//if (_port_type[n] == 1 )
				{
					if ( _port_m[n] == 1 && _port_type[n] == 1 )
					{
						uint8_t j = i + 1;
						if ( j >= strlen(srv_cmd) || srv_cmd[j] == ':' || srv_cmd[j] == ';' )
						{
							pwm[cnt] = srv_cmd[i];
							pwm[cnt + 1] = '\0';
							if ( port_letter == 'D' )
							{
								//if ( port_num == 6 || port_num == 5 || port_num == 3 )
								port_misc_cur[n] = atoi(pwm);

								if ( p_flag == 4 && *w_ocr > 0 )
								port_misc_cur[n] = 0;

								_port_misc[n] = eeprom_read_word((unsigned int*)&ee_port_misc[n]);

								if ( p_flag >= 5 )
								{
									// Клавиша отпущена после короткого нажатия
									if ( port_misc_cur[c_input] == 0 && cmd_delay_p[c_input] == 0 )
									{
										// Нестандартный вариант с одной клавишей
										// когда предыдущее действие было снижение яркости. Короткое нажатие должно выключать нагрузку
										if ( ( *w_ocr > 0 && srv_cmd[i] == '~' ) || p_flag != 5 )
										{
											*w_ocr = 0;
											srv_timeout[n] = 1;
										}
										else if ( p_flag == 5 )
										{
											if ( cmd_delay_p[n] == 0 )
											cmd_delay_p[n] = 255;
											*w_ocr = cmd_delay_p[n];
											srv_timeout[n] = 2;
										}
									}
									// Клавиша удерживается, длительное нажатие
									else if ( port_misc_cur[c_input] == 1 )
									{
										if ( p_flag == 5 && *w_ocr < 255 )
										*w_ocr = *w_ocr + 1;
										else if ( p_flag == 6 && *w_ocr > 0 )
										*w_ocr = *w_ocr - 1;

										cmd_delay_p[c_input] = 1;
										port_misc_cur[c_input] = 3;

										cmd_delay_p[n] = *w_ocr;

									}
									// Клавиша отпущена после длительного нажатия
									else if ( port_misc_cur[c_input] == 0 && cmd_delay_p[c_input] == 1 )
									{

										if ( srv_timeout[n] == 1 || srv_timeout[n] == 0 )
										srv_timeout[n] = 2;
										else
										srv_timeout[n] = 1;
										cmd_delay_p[c_input] = 0;
									}
									else
									cmd_delay_p[c_input] = 0;

								}
								else if ( _port_misc[n] == 0 )
								*w_ocr = port_misc_cur[n];
								else
								port_cnt[n] = _port_misc[n];

							}

							p_num[0] = 'F';
							p_num[1] = 'F';
							pwm[0] = '\0';
							cnt = 0;
						}
						else
						{
							if ( srv_cmd[i] == '*' )
							p_flag = 4;
							else
							{
								pwm[cnt] = srv_cmd[i];
								cnt++;
							}
						}
						
					}
					else
					{
						// ON
						if ( srv_cmd[i] == '1' || ( srv_cmd[i] == '3' && port_state == 1 ) || ( srv_cmd[i] == '4' && port_state == 0 ) )
						*w_port |= (1<<port_num);
						// OFF
						else if ( srv_cmd[i] == '0' || ( srv_cmd[i] == '3' && port_state == 0 ) || ( srv_cmd[i] == '4' && port_state == 1 ) )
						*w_port &= ~(1<<port_num);
						// Toggle
						else if ( srv_cmd[i] == '2' )
						*w_port ^= (1<<port_num);

						p_num[0] = 'F';
						p_num[1] = 'F';
					}
				}
			}
			else if ( p_flag == 2 )
			{
				uint8_t j = i + 1;
				cmd_delay[cnt] = srv_cmd[i];
				cmd_delay[cnt + 1] = '\0';
				cnt++;
				if ( j >= strlen(srv_cmd) || srv_cmd[j] == ':' || srv_cmd[j] == ';' )
				{
					uint8_t j2 = j + 1 + cmd_shift;
					if ( j2 < 23 )
					{
						srv_timeout_act[c_input] = atoi(cmd_delay) * 20;
						cmd_delay_p[c_input] = j2;
						memset(cmd_delay, 0, 5);
						break;
					}
				}
				
			}
			else if ( p_flag == 3 )
			{
				uint8_t j;
				i++;

				for ( j = 0; j < IO_SIZE; j++ )
				{
					port_letter = aio[j][0];
					port_num = atoi(&aio[j][1]);

					sw_port(port_letter);

					if ( _port_type[j] == 1 )
					{
						if ( _port_m[j] == 1 )
						{
							sw_ocr(port_num);
							if ( srv_cmd[i] == '0' )
							*w_ocr = 0;
							/*
							else if ( srv_cmd[i] == '1' )
							*w_ocr = 255;
							else if ( srv_cmd[i] == '2' )
							{
								if ( *w_ocr > 0 )
								*w_ocr = 0;
								else
								*w_ocr = 255;
							}*/
						}
						else
						{
							// ON
							if ( srv_cmd[i] == '1' )
							*w_port |= (1<<port_num);
							// OFF
							else if ( srv_cmd[i] == '0' )
							*w_port &= ~(1<<port_num);
							// Toggle
							else if ( srv_cmd[i] == '2' )
							*w_port ^= (1<<port_num);
						}

					}
				}
			}


		}
	}

}

void browserresult_callback(uint16_t statuscode,uint16_t datapos __attribute__((unused)), uint16_t len __attribute__((unused)), uint8_t c_input)
{
	srv_timeout[c_input] = 0;

	uint16_t i=0;
	uint8_t j=0;
	uint8_t k=0;

	char * srv_rep;
	srv_rep = (char *)&(buf[datapos]);
	char srv_cmd[30];

	for ( i = 0; i < 30; i++ )
	srv_cmd[i] = *"";

	// Предполагается, что в нормальной ситуации ответ сервера не может содержать более 200 байт
	if ( len > 220 )
	len = 220;

	if ( ( _port_type[c_input] == 3 && _port_d[c_input] == 4 ) || c_input >= IO_SIZE  )
	send_eth_flag2 = -1;
	else
	send_eth_flag2 = 1;

	if (statuscode == 200)
	{
		for ( i = 0; i < len; i++ )
		{
			// Пропускаем все HTTP-заголовки
			if ( (*srv_rep == '\r' || *srv_rep == '\n') && k != 2 )
			{
				srv_rep++;
				if ( *srv_rep == '\n' )
				k++;
				continue;
			}

			if ( k == 2 )
			{
				srv_cmd[j] = *srv_rep;
				j++;
				if ( j > 28 )
				break;
			}

			if ( k != 2 )
			k = 0;

			srv_rep++;
		}

		srv_cmd[j] = '\0';

		if (strlen(srv_cmd) > 0)
		port_execute(srv_cmd, 100, 0, 3);
	}
	// Если ответ сервера не 200, то выполняем команды по умолчанию
	else if ( (_port_m[c_input] == 0 && port_act[c_input] == -1) || (_port_m[c_input] == 2 && port_act[c_input] == -2) || _port_m[c_input] == 1 || _port_type[c_input] != 0 )
	{
		memset(temp, 0, sizeof(temp));
		eeprom_read_block (temp, &ee_cmd[c_input], 23);
		if ( *temp != (char)0xff && *temp )
		port_execute(temp, c_input, 0, port_act[c_input] + 2);

		send_eth_flag2 = 2;

	}

}


void browserresult_callback_empty(uint16_t statuscode,uint16_t datapos __attribute__((unused)), uint16_t len __attribute__((unused)), uint8_t c_input __attribute__((unused)))
{
	if (statuscode==200)
	{}
	send_eth_flag2 = -1;
}


void arpresolver_result_callback(uint8_t *ip __attribute__((unused)),uint8_t reference_number,uint8_t *mac){
        uint8_t i=0;
        if (reference_number==TRANS_NUM_GWMAC){
                // copy mac address over:
                while(i<6){gwmac[i]=mac[i];i++;}

        }
	/*
        if (reference_number==TRANS_NUM_WEBMAC){
                // copy mac address over:
                while(i<6){www_gwmac[i]=mac[i];i++;}
        }
	*/
}

void arpresolver_result_callback2(uint8_t *ip __attribute__((unused)),uint8_t reference_number,uint8_t *mac){
        uint8_t i=0;
	      // copy mac address over:
        if (reference_number==TRANS_NUM_WEBMAC){
                while(i<6){www_gwmac2[i]=mac[i];i++;}
	}
}

ISR(TIMER1_COMPA_vect)            //every 5ms
{
	char i;
	static char ct0 = 0xFF, ct1 = 0xFF;  // 8 * 2bit counters
	static char ct2 = 0xFF, ct3 = 0xFF;  // 8 * 2bit counters
	static char ct4 = 0xFF, ct5 = 0xFF;  // 8 * 2bit counters

	uint8_t j;

	if ( adc_check_flag == 1 )
	adc_timer++;

	/*Temp-check*/
	//if ( temp_check_flag > 0 )
	//temp_timer++;

	if ( srv_loop_timer > 2 )
	srv_loop_timer--;

	//Watchdog
	//if ( _preset != 255 && arp_timeout > 10000 )
	//arp_timeout--;

	// Проверка на установленный таймаут каждый порт
	for ( j = 0; j < IO_SIZE; j++ )
	{
		if ( _port_type[j] == 1 )
		{
			if ( _port_m[j] == 1 && _port_misc[j] > 0 && port_cnt[j] > 0 )
			{
				if ( port_cnt[j] == 1 )
				{
					uint8_t k = 0, pnum;
					pnum = atoi(&aio[j][1]);
	
					sw_ocr(pnum);
					k = *w_ocr;

					if ( port_misc_cur[j] > k )
					k++;
					else if ( port_misc_cur[j] < k )
					k--;
					else
					_port_misc[j] = 0;

					*w_ocr = k;

					port_cnt[j] = _port_misc[j];
				}
				else
				port_cnt[j]--;
			}
		}
		else
		{
			if ( srv_timeout[j] > 0 )
			{
				srv_timeout[j]--;
				// Если счетчик дошел до конца, выставляем флаг - выполняем команду по умолчанию
				if ( srv_timeout[j] == 0 )
				srv_timeout_act[j] = 1;
			}

			if ( srv_timeout_act[j] > 2 )
			srv_timeout_act[j]--;

			if ( _port_type[j] == 3 && _port_d[j] == 4 )
			{
				if ( port_misc_cur[j] > 0 )
				port_misc_cur[j]--;
			}

			// Длительное нажатие
			if ( _port_type[j] == 0 && port_misc_cur[j] > 1 )
			port_misc_cur[j]--;

		}

	}

	i = ~PIND;              // read keys (low active)
	i ^= input_state;            // key changed ?
	ct0 = ~(ct0 & i);            // reset or count ct0
	ct1 = ct0 ^ (ct1 & i);       // reset or count ct1
	i &= ct0 & ct1;              // count until roll over ?
	input_release |= input_state & i;  // 1->0: key release detect 
	input_state ^= i;            // then toggle debounced state
	input_press |= input_state & i;   // 0->1: key press detect

	i = ~PINC;              // read keys (low active)
	i ^= input_state2;            // key changed ?
	ct2 = ~(ct2 & i);            // reset or count ct0
	ct3 = ct2 ^ (ct3 & i);       // reset or count ct1
	i &= ct2 & ct3;              // count until roll over ?
	input_release2 |= input_state2 & i;  // 1->0: key release detect 
	input_state2 ^= i;            // then toggle debounced state
	input_press2 |= input_state2 & i;   // 0->1: key press detect

	i = ~PINB;              // read keys (low active)
	i ^= input_state3;            // key changed ?
	ct4 = ~(ct4 & i);            // reset or count ct0
	ct5 = ct4 ^ (ct5 & i);       // reset or count ct1
	i &= ct4 & ct5;              // count until roll over ?
	input_release3 |= input_state3 & i;  // 1->0: key release detect 
	input_state3 ^= i;            // then toggle debounced state
	input_press3 |= input_state3 & i;   // 0->1: key press detect

	isr_flag = 1;
} 

char get_key_press( char input_mask )
{
	cli();
	input_mask &= input_press;                // read key(s)
	input_press ^= input_mask;                // clear key(s)
	sei();
	return input_mask;
}

char get_key_release( char input_mask )
{
	cli();
	input_mask &= input_release;                // read key(s)
	input_release ^= input_mask;                // clear key(s)
	sei();
	return input_mask;
}

char get_key_press2( char input_mask )
{
	cli();
	input_mask &= input_press2;                // read key(s)
	input_press2 ^= input_mask;                // clear key(s)
	sei();
	return input_mask;
}

char get_key_release2( char input_mask )
{
	cli();
	input_mask &= input_release2;                // read key(s)
	input_release2 ^= input_mask;                // clear key(s)
	sei();
	return input_mask;
}

char get_key_press3( char input_mask )
{
	cli();
	input_mask &= input_press3;                // read key(s)
	input_press3 ^= input_mask;                // clear key(s)
	sei();
	return input_mask;
}

char get_key_release3( char input_mask )
{
	cli();
	input_mask &= input_release3;                // read key(s)
	input_release3 ^= input_mask;                // clear key(s)
	sei();
	return input_mask;
}

void delay_us(uint16_t count)
{
	while(count--)
	_delay_us(20);
}

void sw_dir(char dir, char port_letter, uint8_t port_num)
{
	sw_port(port_letter);

	if ( dir == '1' )
	*w_ddr |= (1<<port_num);
	else
	*w_ddr &= ~(1<<port_num);
}

int main(void)
{
	uint16_t plen;
	uint16_t dat_p;
	int8_t cmd;

	uint8_t _eth_addr[4];
	uint16_t plen_start = 0;
/*
	typedef struct  
	{
		uint8_t header;
		uint8_t msg_id;
		uint8_t cmd;
		uint8_t data[14];
	} udp_data_struct;
	#define check_new_UDP_message() ((plen>0) && (buf[IP_PROTO_P]==IP_PROTO_UDP_V))
	*/

	//wdt_disable();

	CLKPR=(1<<CLKPCE); // change enable
	CLKPR=0; // "no pre-scaler"
	_delay_loop_1(0); // 60us

	// Reading EEPROM IP address
	//eeprom_read_block ((void *)_ip_addr, (const void *)&ee_ip_addr,4);
	eeprom_read_block ((void *)_ip_addr, (const void *)ee_ip_addr,4);
	if ( _ip_addr[0] != 255 )
	{
		/*
		for ( i = 0; i < 4; i++ )
		{
			myip[i] = _ip_addr[i];
			mymac[i+2] = _ip_addr[i];
		}
		*/

		// Развернутый цикл занимает меньше памяти и выполняется быстрее...
		myip[0] = _ip_addr[0];
		mymac[2] = _ip_addr[0];
		myip[1] = _ip_addr[1];
		mymac[3] = _ip_addr[1];
		myip[2] = _ip_addr[2];
		mymac[4] = _ip_addr[2];
		myip[3] = _ip_addr[3];
		mymac[5] = _ip_addr[3];

	}

	/*initialize enc28j60*/
	enc28j60Init(mymac);
	enc28j60clkout(2); // change clkout from 6.25MHz to 12.5MHz
	_delay_loop_1(0); // 60us

	/* Magjack leds configuration, see enc28j60 datasheet, page 11 */
	// LEDB=yellow LEDA=green
	// 0x476 is PHLCON LEDA=links status, LEDB=receive/transmit
	enc28j60PhyWrite(PHLCON,0x476);

	eeprom_read_block (_port_type, &ee_port_type, IO_SIZE);
	eeprom_read_block (_port_d, &ee_port_d, IO_SIZE);
	eeprom_read_block (_port_m, &ee_port_m, IO_SIZE);
	eeprom_read_block (_port_misc, &ee_port_misc, IO_SIZE * 2);

	_preset = eeprom_read_byte(&ee_preset);
	_alarm_temp = eeprom_read_byte(&ee_alarm_temp);

	_srv_port = eeprom_read_word(&ee_srv_port);
	if ( _srv_port == 65535 )
	_srv_port = 80;

	eeprom_read_block (password, &ee_pwd, 6);
	// Default password
	if ( password[0] == (char)255 )
	strcpy(password, "sec");

	DDRB|= (1<<DDB0); // enable PB0, LED as output 

	// Enable ADC
	// Set ADC prescalar
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1);
	// Set ADC reference to AVCC 
	ADMUX |= (1 << REFS0);
	// Left adjust ADC result to allow easy 8 bit reading 
	//ADMUX |= (1 << ADLAR); 
	// Enable ADC 
	ADCSRA |= (1<<ADEN);
	// Start A2D Conversions 
	//ADCSRA |= (1<<ADSC);

	for ( i = 0; i < IO_SIZE; i++ )
	{
		port_letter = aio[i][0];
		port_num = atoi(&aio[i][1]);
		srv_timeout[i] = 0;
		srv_timeout_act[i] = 0;
		port_misc_cur[i] = 0;

		sw_port(port_letter);

		if ( _port_type[i] == 1 )
		{

			*w_ddr |= (1<<port_num);
			if (_port_d[i] == 1 )
			*w_port |= (1<<port_num);
			else
			*w_port &= ~(1<<port_num);

			if ( port_letter == 'D' )
			{
				// Выключаем ШИМ
				if ( port_num == 6 )
				TCCR0A &= ~(1 << COM0A1);
				else if ( port_num == 5 )
				TCCR0A &= ~(1 << COM0B1);
				else if ( port_num == 3 )
				TCCR2A &= ~(1 << COM2B1);
			}

			// Включаем ШИМ
			if ( _port_m[i] == 1 )
			{
				if ( port_letter == 'D' )
				{
					TCCR0B |= (1 << CS01);

					if ( port_num == 6 )
					{
						OCR0A = 0;
						TCCR0A |= (1 << COM0A1);
						// set Phase Correct PWM Mode
						TCCR0A |= (0 << WGM01) | (1 << WGM00);
						// set fast PWM Mode
						//TCCR0A |= (1 << WGM01) | (1 << WGM00);

						//TCCR0B |= (1 << CS01);

					}

					if ( port_num == 5 )
					{
						OCR0B = 0;
						TCCR0A |= (1 << COM0B1);
						// set Phase Correct PWM Mode
						TCCR0A |= (0 << WGM01) | (1 << WGM00);
						// set fast PWM Mode
						//TCCR0A |= (1 << WGM01) | (1 << WGM00);

						//TCCR0B |= (1 << CS01);
					}

					if ( port_num == 3 )
					{
						OCR2B = 0;
						TCCR2A |= (1 << COM2B1);
						// set Phase Correct PWM Mode
						TCCR2A |= (0 << WGM21) | (1 << WGM20);
						// set fast PWM Mode
						//TCCR2A |= (1 << WGM21) | (1 << WGM20);
						TCCR2B |= (1 << CS21);

					}

					sw_ocr(port_num);
					*w_ocr = _port_d[i];

				}
			}
		}
		else if ( _port_type[i] != 255 || port_letter == 'A' )
		{
			if ( port_letter != 'A' )
			{
				if ( _port_type[i] == 3 && _port_d[i] == 3 )
				{
					ds18b20_gettemp(port_num);
					port_misc_cur[i] = _port_misc[i];
				}
				else
				*w_ddr &= ~(1<<port_num);
			}

			if ( ((_port_type[i] == 2 || port_letter == 'A') && _port_m[i] > 0 && _port_m[i] < 255 ) || (_port_type[i] == 3 && _port_d[i] == 3 ) )
			adc_check_flag = 1;

		}

		// Hyst
		if ( _port_type[i] == 2 || port_letter == 'A' || _port_type[i] == 3 )
		{
			port_cnt[i] = eeprom_read_word(&ee_port_hyst[i]);
			if ( port_cnt[i] == 65535 )
			port_cnt[i] = 0;
		}


	}

	// Debouncing init
	TCCR1B |=   (1 << WGM12);         //Timer1 Mode 2: CTC
	TCCR1B |=   (1 << CS12);          //Divide by 256
	OCR1A    =   F_CPU / 512 * 10e-3; //Set CTC compare value to 5ms at 12,5 MHz AVR clock, with a prescaler of 256
	// (1/200Hz = 5ms) -> (12.5Mhz/512*0.01)*256 = 62500 циклов. 12.5Mhz/62500 циклов = 200Гц
	TIMSK1 =   (1 << OCIE1A);         //Enable T1 interrupt
	//input_state    = ~PIND;      //no action on keypress during reset
	sei();

	// Инициализируем сетевые настройки
	// --- init_ip_arp_udp_tcp(mymac,myip,HTTPPORT);
        init_mac(mymac);
        client_ifconfig(myip,netmask);

	// Reading Gateway / Server IP address
	eeprom_read_block ((void *)_sip_addr, (const void *)&ee_gw_addr,4);
	if ( _sip_addr[0] == 255 )
	eeprom_read_block ((void *)_sip_addr, (const void *)&ee_sip_addr,4);
	// Reading script-name
	eeprom_read_block (_spt, &ee_spt, 15);
	// Reading mdid
	eeprom_read_block (_mdid, &ee_mdid, 5);
	if ( strlen(_mdid) == 0 )
	_mdid[0] = 255;

	if ( _sip_addr[0] != 255 && _sip_addr[0] > 0 )
	{
		LEDON;
		get_mac_with_arp(_sip_addr,TRANS_NUM_GWMAC,&arpresolver_result_callback);

		// Предотвращение зацикливания в случае неудачи ARP-резолвинга IP-адреса сервера
		arp_timeout = 9000;
                while(get_mac_with_arp_wait() && arp_timeout > 0 )
		{
                        // to process the ARP reply we must call the packetloop
                        plen=enc28j60PacketReceive(BUFFER_SIZE, buf);
                        packetloop_arp_icmp_tcp(buf,plen);
			arp_timeout--;
                }
		
		//arp_timeout = 32000;

		//get_mac_with_arp(_sip_addr,TRANS_NUM_WEBMAC,&arpresolver_result_callback);

		//gw_set = 1;
		LEDOFF;

		// Для экономии SRAM используем переменную _sip_addr как для Gateway IP, так и для Server IP
		eeprom_read_block ((void *)_sip_addr, (const void *)&ee_sip_addr,4);

	}

	/*Temp-check*/
	//temp_check_flag = eeprom_read_byte(&ee_temp_check);
	//if ( temp_check_flag == 1 && ( _alarm_temp == 255 || _sip_addr[0] == 255 || _sip_addr[0] == 0 ) )
	//temp_check_flag = 0;

	//srv_loop_timer = eeprom_read_byte(&ee_srv_loop) * SRV_LOOP_TIMEOUT;

	srv_loop_timer = eeprom_read_byte(&ee_srv_loop);
	if ( srv_loop_timer == 1 )
	srv_loop_timer = SRV_LOOP_TIMEOUT;
	else
	srv_loop_timer = 0;

        //www_server_port(HTTPPORT);

	if ( _sip_addr[0] != 255 && enc28j60linkup() )
	{
		strcpy(temp, _spt);
		strcat(temp, "?st=1");
		client_browse_url(PSTR("/"),temp,shost,&browserresult_callback_empty, _sip_addr, gwmac, _srv_port);
	}

	wdt_enable(WDTO_4S);

	while(1)
	{
		wdt_reset();

		plen=enc28j60PacketReceive(BUFFER_SIZE, buf);

		if ( buffer_overflow == 1 )
		{
			enc28j60Init(mymac);
			buffer_overflow = 0;
		}

		buf[BUFFER_SIZE]='\0';
		//--- dat_p=packetloop_icmp_tcp(buf,plen);
		enc28j60EnableBroadcast(); 
        dat_p=packetloop_arp_icmp_tcp(buf,plen);

		// Отправка информации по доп. URL. Только в том случае, если завершена TCP-сессия с основным сервером. (ограничения tuxgraphics TCP/IP)
		// Проблема. Если в момент таймаута сработал еще один вход, то отработает последний сработавший. Массив и очередь?
		if ( send_eth_flag2 > -1 )
		{
			// Вызываем URL только если NetAction flag не установлен или если установлен и сервер не отвечает или сервер не прописан
			if ( (tcp_client_state == 0 || tcp_client_state >= 5) )
			{

				if ( eeprom_read_byte(&ee_netact_flag[cur_input]) != 1 || (eeprom_read_byte(&ee_netact_flag[cur_input]) == 1 && send_eth_flag2 == 2 ) || _sip_addr[0] == 255 )
				{
					/*
					//&&&
		                        // to process the ARP reply we must call the packetloop
		                        plen=enc28j60PacketReceive(BUFFER_SIZE, buf);
		                        packetloop_arp_icmp_tcp(buf,plen);
					arp_timeout--;
					*/

					eeprom_read_block (_eth_addr, &ee_eth_cmd[cur_input], 4);

					if ( _eth_addr[0] != 255 ) 
					{
						uint16_t _na_port = 80;

						memset(temp, 0, sizeof(temp));
						//eeprom_read_block (temp, (void*)ee_eth_cmd[cur_input] + 4, 13);
						if ( eeprom_read_byte((void*)ee_eth_cmd[cur_input] + 4) == ':' )
						{
							_na_port = eeprom_read_word((void*)ee_eth_cmd[cur_input] + 5);
							eeprom_read_block (temp, (void*)ee_eth_cmd[cur_input] + 7, 10);
						}
						else
						eeprom_read_block (temp, (void*)ee_eth_cmd[cur_input] + 4, 13);

						get_mac_with_arp(_eth_addr,TRANS_NUM_WEBMAC,&arpresolver_result_callback2);

						arp_timeout = 5000;
				                while(get_mac_with_arp_wait() && arp_timeout > 0 )
						{
				                        // to process the ARP reply we must call the packetloop
				                        plen=enc28j60PacketReceive(BUFFER_SIZE, buf);
				                        packetloop_arp_icmp_tcp(buf,plen);
							arp_timeout--;
				                }

						// Если NetAction похож на команду другому устройству и используются команды 3 и 4, меняем их на нужные
						// aka "Удаленный термостат"
						if ( temp[strlen(temp)-2] == ':' && port_act[cur_input] < 0 )
						{
							if ( temp[strlen(temp)-1] == '3' || temp[strlen(temp)-1] == '4' )
							{
								if ( (port_act[cur_input] == -1 && temp[strlen(temp)-1] == '3') || (port_act[cur_input] == -2 && temp[strlen(temp)-1] == '4') )
								temp[strlen(temp)-1] = '1';
								else
								temp[strlen(temp)-1] = '0';
							}
						}

						if ( _mdid[0] != (char)255 && _mdid[0] != '\0' )
						{
							if ( strchr(temp, '?') )
							strcat(temp, "&");
							else
							strcat(temp, "?");
							strcat(temp, "mdid=");
							strcat(temp, _mdid);
						}

						sprintf(shost,"%d.%d.%d.%d",_eth_addr[0],_eth_addr[1],_eth_addr[2],_eth_addr[3]);
						LEDON;
						client_browse_url(PSTR("/"),temp,shost,&browserresult_callback_empty, _eth_addr, www_gwmac2, _na_port);
						LEDOFF;
	
					}

				}

				//else
				send_eth_flag2 = -1;

			}
		}

		/*
		// Watchdog
		if ( _preset != 255 && _sip_addr[0] != 255 )
		{		
			if ( (tcp_client_state == 0 || tcp_client_state >= 5) && arp_timeout == 10000 )
			get_mac_with_arp(_sip_addr,TRANS_NUM_GWMAC,&arpresolver_result_callback);

			if ( arp_timeout <= 10000 )
			{
        			arp_timeout--;
		
				if ( get_mac_with_arp_wait() == 0 )
				{
					arp_timeout = 32000;
					//LEDOFF;
					//break;
				}
				
				if ( arp_timeout <= 5005 )
				{
					//LEDON;
					arp_timeout = 32000;

					eeprom_read_block (temp, &ee_cmd[_preset], 23);
					//if ( temp[0] != (char)255 && strlen(temp) > 0 ) 
					if ( *temp != (char)0xff && *temp )
					port_execute(temp, _preset, 0, 1);

				}
			}
		}
		*/

		// Проверка состояния входов
		for ( i = 0; i < IO_SIZE; i++ )
		{
			//port_letter = aio[i][0];
			//port_num = atoi(&aio[i][1]);

			// Если порт не OUT
			//if ( _port_type[i] == 0 || ( _port_type[i] == 2 || port_letter == 'A') || ( _port_type[i] == 3 && _port_d[i] == 3 ) )
			if ( _port_type[i] != 1 )
			{
				port_letter = aio[i][0];
				port_num = atoi(&aio[i][1]);


				// Таймаут завершен, выполняем команды по умолчанию
				if ( srv_timeout_act[i] == 1 )
				{
					srv_timeout_act[i] = 0;

					memset(temp, 0, sizeof(temp));
					eeprom_read_block (temp, &ee_cmd[i], 23);
					//if ( temp[0] != (char)255 && strlen(temp) > 0 ) 
					if ( *temp != (char)0xff && *temp )
					port_execute(temp, i, 0, port_act[i] + 2);

					// Если с текущим входом связан доп. адрес, ставим флаг для проверки записанного в EEPROM URL
					// Значение 2 - нужно переопределить default gateway, поскольку основной сервер не отвечает
					tcp_client_state = 0;
					send_eth_flag2 = 2;
					//&&&
					gwmac[0] = 255;

				}

				if ( _port_type[i] == 0 || ( ((_port_type[i] == 2 || port_letter == 'A') || (_port_type[i] == 3 && _port_d[i] >= 3)) && port_act[i] > 0 ) )
				{

					char my_mask = '\0';
					char my_mask2 = '\0';
	
					if ( _port_type[i] == 0 )
					{
						// Обработка нажатия кнопки.
						// Для режима RAW (без защиты от требезга)
						/*
						if ( _port_d[i] == 1 )
						{
							if (port_letter == 'D')
							my_mask = !(PIND & (1<<port_num));
							else if (port_letter == 'C')
							my_mask = !(PINC & (1<<port_num));
							else
							my_mask = !(PINB & (1<<port_num));
						}
						*/
						if ( _port_d[i] == 1 || _port_d[i] == 7 )
						{
							if (port_letter == 'D')
							my_mask = !(PIND & (1<<port_num));
							else if (port_letter == 'C')
							my_mask = !(PINC & (1<<port_num));
							else
							my_mask = !(PINB & (1<<port_num));

							//=================
							if ( _port_d[i] == 1 && my_mask )
							_port_d[i] = 7;
							else if ( _port_d[i] == 7  )
							{
								if ( my_mask )
								my_mask = '\0';
								else
								{
									my_mask2 = 0xFF;
									_port_d[i] = 1;
								}
							}
							else if ( _port_m[i] == 2 && my_mask )
							_port_d[i] = 7;
							//=================
						}

						// C защитой от требезга
						else
						{
							if (port_letter == 'D')
							{
								my_mask = get_key_press(1 << port_num);
								my_mask2 = get_key_release(1 << port_num);
							}
							else if (port_letter == 'C')
							{
								my_mask = get_key_press2(1 << port_num);
								my_mask2 = get_key_release2(1 << port_num);
							}
							else
							{
								my_mask = get_key_press3(1 << port_num);
								my_mask2 = get_key_release3(1 << port_num);
							}
						}
					}
					// DS18B20 + команды 3 и 4
					else if ( (_port_d[i] == 3 || _port_type[i] == 2 || port_letter == 'A') && port_act[i] == 1 )
					my_mask = 0xFF;
	
					// Длительное нажатие - сброс счетчика при отпускании клавиши
					// if ( my_mask2 )
					if ( my_mask2 && _port_type[i] == 0 ) // UPDATE*
					port_misc_cur[i] = 0;

					if ( port_act[i] > 0 || ( ( (_port_m[i] == 0 || _port_m[i] == 255) && my_mask ) || ( _port_m[i] == 1 && (my_mask || my_mask2) ) || ( _port_type[i] == 0 && _port_misc[i] == 1 && (my_mask || my_mask2) && _sip_addr[0] != 255 && enc28j60linkup() ) || ( _port_m[i] == 2 && my_mask2 ) ) || srv_timeout[i] > 150 )
					{

						// Сохраняем тип события (P или R) - иначе после завершения таймаута мы уже не определим его
						/*
						if ( my_mask2 )
						port_act[i] = -2;
						else if ( my_mask )
						port_act[i] = -1;
						*/
						if ( _port_d[i] == 3 && _port_type[i] == 3 )
						{
							if ( my_mask )
							port_act[i] = -1;
							else
							port_act[i] = -2;
						}
						else
						{
							if ( my_mask2 )
							port_act[i] = -2;
							else if ( my_mask )
							port_act[i] = -1;
							else
							port_act[i] = -3;
						}

						//if ( (((_port_m[i] == 0  || _port_m[i] == 255 ) && my_mask ) || (_port_m[i] == 2 && my_mask2) || _port_m[i] == 1) && srv_timeout[i] < 150 && port_misc_cur[i] != 1 ) 
						if ( (((_port_m[i] == 0  || _port_m[i] == 255 ) && my_mask ) || (_port_m[i] == 2 && my_mask2) || _port_m[i] == 1) && srv_timeout[i] < 150 && port_misc_cur[i] != 1 && _port_type[i] == 0 ) 
						port_cnt[i]++;

						// Длительное нажатие
						if ( _port_type[i] == 0 && _port_m[i] != 2 && my_mask && port_misc_cur[i] <= 0 )
						port_misc_cur[i] = 200;

						// Если прописан основной сервер
						if ( _sip_addr[0] != 255 && enc28j60linkup() )
						{
							//&&&
							// Устройство загрузилось раньше роутера. Нужно сбросить статус соединения, если оно активно.
							if ( gwmac[0] == 255 )
							tcp_client_state = 0;

							// Нельзя установить две TCP-сессии с сервером одновременно, поэтому если какая-либо сессия активна, ждем ее завершения
							if ( (tcp_client_state == 0 || tcp_client_state >= 5) )
							{
								//&&&
								//if ( get_mac_with_arp_wait() )
								if ( gwmac[0] == 255 )
								{
									eeprom_read_block ((void *)_sip_addr, (const void *)&ee_gw_addr,4);
									if ( _sip_addr[0] == 255 )
									eeprom_read_block ((void *)_sip_addr, (const void *)&ee_sip_addr,4);

									get_mac_with_arp(_sip_addr,TRANS_NUM_GWMAC,&arpresolver_result_callback);

									// Предотвращение зацикливания в случае неудачи ARP-резолвинга IP-адреса сервера
									arp_timeout = 5000;
							        while(get_mac_with_arp_wait() && arp_timeout > 0 )
									{
							                        // to process the ARP reply we must call the packetloop
							                        plen=enc28j60PacketReceive(BUFFER_SIZE, buf);
							                        packetloop_arp_icmp_tcp(buf,plen);
										arp_timeout--;
							        }

									//arp_timeout = 0;
									eeprom_read_block ((void *)_sip_addr, (const void *)&ee_sip_addr,4);
								}


								//int j = 0;
								int8_t j = 0;
								char pnum[3];
								char adc_val[5];
								itoa(i, pnum, 10);

								sprintf(shost,"%d.%d.%d.%d",_sip_addr[0],_sip_addr[1],_sip_addr[2],_sip_addr[3]);
								strcpy(temp, _spt);
								strcat(temp, "?pt=");
								strcat(temp, pnum);

								if ( _port_type[i] == 0 )
								{
									//if ( my_mask2 || port_act[i] == -2 )
									if ( port_act[i] == -2 )
									strcat(temp, "&m=1");

									// Длительное нажатие
									else if ( port_misc_cur[i] == 1 )
									{
										strcat(temp, "&m=2");
										port_misc_cur[i] = 0;
										port_act[i] = 0;
									}

									//strcat(temp, "&cnt=");
									//sprintf(adc_val, "%u", port_cnt[i]);
									//strcat(temp, adc_val);
									sprintf(temp, "%s&cnt=%u", temp, port_cnt[i]);
									//snprintf_P(temp, sizeof(temp), PSTR("%s&cnt=%u"), temp, port_cnt[i]);

								}

								else if ( ( _port_type[i] == 2 || ( _port_type[i] == 3 && _port_d[i] == 3 ) || port_letter == 'A' ) )
								{
									strcat(temp, "&v=");
									itoa(port_misc_cur[i], adc_val, 10);
									strcat(temp, adc_val);
									strcat(temp, "&dir=");
									if ( port_misc_cur[i] > _port_misc[i] )
									strcat(temp, "1");
									else
									strcat(temp, "0");

									if ( _port_type[i] == 2 )
									port_act[i] = 0;

								}
								else if ( _port_type[i] == 3 && _port_d[i] == 4 )
								{
									sw_port(port_letter);

									ds18b20_writebyte(DS18B20_CMD_READROM, port_num);
									uint8_t crc = crc8(0, ds18b20_readbyte(port_num));
									{
										strcat(temp, "&ib=");
										j = 6;
										while(j--)
										{
											int inbyte = ds18b20_readbyte(port_num);

											sprintf(adc_val, "%02x", inbyte);
											strcat(temp, adc_val);

											crc = crc8(crc, inbyte);
										}

										if ( crc == ds18b20_readbyte(port_num) && crc != 0 )
										{
											port_misc_cur[i] = 150;
											j = 0;
										}
										else
										j = 1;
									}

								}
								/*
								else
								{
									strcat(temp, "&cnt=");
									sprintf(adc_val, "%u", port_cnt[i]);
									strcat(temp, adc_val);
								}
								*/

								if ( _mdid[0] != (char)255 )
								{
									strcat(temp, "&mdid=");
									strcat(temp, _mdid);
								}


								cur_input = i;

								if ( j == 0 )
								{
									LEDON;
									client_browse_url(PSTR("/"),temp,shost,&browserresult_callback, _sip_addr, gwmac, _srv_port);
									LEDOFF;
								}

								if ( (_port_m[i] == 0 && port_act[i] == -1) || (_port_m[i] == 2 && port_act[i] == -2) || _port_m[i] == 1 || _port_type[i] != 0 ) 
								srv_timeout[i] = 150;

								if ( srv_loop_timer < 1000 && srv_loop_timer > 0 )
								srv_loop_timer = 1000;

							}
							else if ( srv_timeout[i] == 0 )
							srv_timeout[i] = 250;

							if ( _port_type[i] == 3 && _port_d[i] == 4 )
							port_act[i] = 0;

						}
						// Если сервер не прописан
						else if ( _sip_addr[0] == 255 || !enc28j60linkup() )
						{
							// Если с текущим входом связан доп. адрес, ставим флаг для проверки записанного в EEPROM URL
							if ( port_misc_cur[i] != 1 )
							send_eth_flag2 = 1;

							if ( _port_type[i] == 3 && _port_d[i] != 3 )
							port_act[i] = 0;

							cur_input = i;
							memset(temp, 0, sizeof(temp));
							eeprom_read_block (temp, &ee_cmd[i], 23);
							// Выполняем команды, если в EEPROM есть реальная информация
							if ( *temp != (char)0xff && *temp )
							{
								if ( my_mask )
								port_execute(temp, i, 0, 1);
								else
								port_execute(temp, i, 0, 0);
							}
						}
					}
				}

				if ( srv_timeout_act[i] == 2 )
				{
					srv_timeout_act[i] = 0;

					char temp2[23];
					//memset(temp, 0, sizeof(temp));
					eeprom_read_block(temp2, (void*)ee_cmd[i] + cmd_delay_p[i], 23 - cmd_delay_p[i]);
					//port_execute(temp2, i, cmd_delay_p[i], port_misc_cur[i]);

					if ( port_act[i] == -3 )
					port_act[i]++;

					port_execute(temp2, i, cmd_delay_p[i], port_act[i] + 2);
				}

				// Длительное нажатие
				if ( _port_type[i] == 0 && _port_m[i] != 2 && port_misc_cur[i] == 1 )
				port_act[i] = 1;

				else if ( _port_type[i] == 3 && _port_d[i] == 4 && port_misc_cur[i] == 0 )
				//if ( port_act[i] == 0 && port_misc_cur[i] == 0 )
				{
					sw_port(port_letter);
					if ( ds18b20_reset(port_num) == 0 ) // presence check
					port_act[i] = 1;
					else
					port_misc_cur[i] = 1;
				}
			}
		}

		if ( adc_check_flag == 1 )
		{
			if ( adc_timer > 1000 )
			{
				for ( i = 0; i < IO_SIZE; i++ )
				{
					port_letter = aio[i][0];
					port_num = atoi(&aio[i][1]);

					if ( _port_type[i] == 2 || ( _port_type[i] == 3 && _port_d[i] == 3 ) || port_letter == 'A' )
					{
						if ( (_port_misc[i] < 1024 && _port_m[i] != 0 ) || _port_type[i] == 3 )
						{
							int16_t my_val;

							if ( _port_type[i] == 3 )
							{
								sw_port(port_letter);

								my_val = ds18b20_gettemp(port_num);
								if ( my_val == 8500 )
								my_val = port_misc_cur[i];
							}
							else
							{
								ADMUX = (1 << REFS0) + port_num; 
								ADCSRA|=(1<<ADSC);
								while (bit_is_set(ADCSRA, ADSC));
								uint8_t my_val_low;
								my_val_low = ADCL;
								my_val = (ADCH<<8)|my_val_low;
							}
	
							//if ( (_port_m[i] == 1 && my_val > _port_misc[i] && port_misc_cur[i] <= _port_misc[i] ) || (_port_m[i] == 2 && my_val < _port_misc[i] && port_misc_cur[i] >= _port_misc[i] ) || ( _port_m[i] == 3 && ( (my_val > _port_misc[i] && port_misc_cur[i] <= _port_misc[i]) || (my_val < _port_misc[i] && port_misc_cur[i] >= _port_misc[i] ) ) ) )
							if ( (_port_m[i] == 1 && my_val > _port_misc[i] + (int16_t) port_cnt[i] && port_misc_cur[i] <= _port_misc[i] + (int16_t) port_cnt[i] ) || (_port_m[i] == 2 && my_val < _port_misc[i] - (int16_t) port_cnt[i] && port_misc_cur[i] >= _port_misc[i] - (int16_t) port_cnt[i] ) || ( _port_m[i] == 3 && ( (my_val > _port_misc[i] + (int16_t) port_cnt[i] && port_misc_cur[i] <= _port_misc[i] + (int16_t) port_cnt[i]) || (my_val < _port_misc[i] - (int16_t) port_cnt[i] && port_misc_cur[i] >= _port_misc[i] - (int16_t) port_cnt[i] ) ) ) )
							{

								/*
								if ( my_val > _port_misc[i] )
								port_act[i] = 1;
								else
								port_act[i] = 2;
								*/
								port_act[i] = 2;
								if ( my_val > _port_misc[i] )
								port_act[i] = 1;
					
							}

							port_misc_cur[i] = my_val;
						}
					}
				}

				adc_timer = 0;
			}
		}

		/*Temp-check*/
		/*
		if ( temp_check_flag > 0 )
		{
			if ( temp_timer > 12000 )
			{
				int8_t cur_temp = readchiptemp();
				char temp2[4];

				if ( cur_temp >= _alarm_temp )
				{
					sprintf(shost,"%d.%d.%d.%d",_sip_addr[0],_sip_addr[1],_sip_addr[2],_sip_addr[3]);
					strcpy(temp, _spt);
					strcat(temp, "?at=");
					sprintf(temp2,"%d", readchiptemp());
					strcat(temp, temp2);

					if ( _mdid[0] != (char)255 )
					{
						strcat(temp, "&mdid=");
						strcat(temp, _mdid);
					}

					LEDON;
					cur_input = IO_SIZE;
					client_browse_url(PSTR("/"),temp,shost,&browserresult_callback, _sip_addr, gwmac, _srv_port);
					LEDOFF;
				}
		
				temp_timer = 0;
			}
		}
		*/

		if ( srv_loop_timer == 1 )
		{

			if (tcp_client_state == 0 || tcp_client_state >= 5)
			{
				for ( i = 0; i < 100; i++ )
				{
					if ( buf[i+TCP_CHECKSUM_L_P+3+plen_start] == 42 )
					buf[700+i] = '\0';
					else
					//break;

					buf[700+i] = buf[i+TCP_CHECKSUM_L_P+3+plen_start];

				}

				//&&&
				if ( gwmac[0] == 255 )
				{
					eeprom_read_block ((void *)_sip_addr, (const void *)&ee_gw_addr,4);
					if ( _sip_addr[0] == 255 )
					eeprom_read_block ((void *)_sip_addr, (const void *)&ee_sip_addr,4);

					get_mac_with_arp(_sip_addr,TRANS_NUM_GWMAC,&arpresolver_result_callback);

					arp_timeout = 5000;
			                while(get_mac_with_arp_wait() && arp_timeout > 0 )
					{
			                        // to process the ARP reply we must call the packetloop
			                        plen=enc28j60PacketReceive(BUFFER_SIZE, buf);
			                        packetloop_arp_icmp_tcp(buf,plen);
						arp_timeout--;
			                }
					//arp_timeout = 0;
					eeprom_read_block ((void *)_sip_addr, (const void *)&ee_sip_addr,4);
				}

				//&&&
				sprintf(shost,"%d.%d.%d.%d",_sip_addr[0],_sip_addr[1],_sip_addr[2],_sip_addr[3]);

				LEDON;
				cur_input = IO_SIZE;
				client_browse_url(PSTR("/"),(char*)&(buf[700]),shost,&browserresult_callback, _sip_addr, gwmac, _srv_port);
				LEDOFF;

				srv_loop_timer = SRV_LOOP_TIMEOUT;
				plen_start = 0;

			}
			else
			{
				srv_loop_timer = 1000;
				srv_loop_timeout++;
			}

			if ( srv_loop_timeout == 6 )
			{
				srv_loop_timeout = 0;
				tcp_client_state = 0;
				gwmac[0] = 255;
			}

		}
		
		

		if ( srv_loop_timer != 2 )
		{
	         if(dat_p==0)
			{
	                        // check if udp otherwise continue
				//uint8_t broadcastmac[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}; 
					
#ifdef ENABLE_UDP_SEARCH
		if (buf[IP_PROTO_P]==IP_PROTO_UDP_V&&buf[UDP_DST_PORT_H_P]==(char)(CONFIG_CLIENT_PORT>>8)&&buf[UDP_DST_PORT_L_P]==(char)(CONFIG_CLIENT_PORT)){

			if(strncmp_P((char *)&buf[UDP_DATA_P],PSTR("SRCH"),4) == 0)
			{
				memcpy_P((unsigned char*)temp, PSTR("MDWM"), 4);
				
				// Reading Gateway / Server IP address
				eeprom_read_block ((void *)_sip_addr, (const void *)&ee_gw_addr,4);			
				memcpy((unsigned char*)temp+4 ,_sip_addr, 4);
				memcpy((unsigned char*)temp+8 ,netmask, 4);

				eeprom_read_block ((void *)_sip_addr, (const void *)&ee_sip_addr,4);					
				
				memcpy((unsigned char*)temp+12,myip, 4);
				memcpy((unsigned char*)temp+16,mymac, 6);
				temp[22]=0;
				temp[23]=0;
				temp[24]=0;
				temp[25]=0;
				send_udp_prepare(buf,CONFIG_CLIENT_PORT,&buf[IP_SRC_P],CONFIG_SERVER_PORT, &buf[ETH_SRC_MAC]);
				memcpy(&buf[UDP_DATA_P],temp, 26);
				send_udp_transmit(buf,26);
				goto UDP;
			}			
			else if(strncmp_P((char *)&buf[UDP_DATA_P],PSTR("SETQ"),4) == 0) // Изменение конфигурации
			{					
				if (memcmp(&buf[UDP_DATA_P+16], mymac, 6) == 0)				
				{
					memcpy_P((unsigned char*)temp, PSTR("SETR"), 4);
					send_udp_prepare(buf,CONFIG_CLIENT_PORT,&buf[IP_SRC_P],CONFIG_SERVER_PORT, &buf[ETH_SRC_MAC]);
					memcpy(&buf[UDP_DATA_P],temp, 4);
					send_udp_transmit(buf,4);
					
					
					eeprom_write_block (&buf[UDP_DATA_P+4],  (void *)&ee_gw_addr,4); //gw
					eeprom_write_block (&buf[UDP_DATA_P+12], (void *)ee_ip_addr, 4); // ip

					RESET();
				}
				
			}
		}

#endif					
/*
				udp_data_struct* udp_data = (udp_data_struct*) (buf + UDP_DATA_P);

			    if ( check_new_UDP_message() && (udp_data->header == 0xAA) )
				{
					send_udp_prepare(buf,52000,&buf[IP_SRC_P],42000, &buf[ETH_SRC_MAC]);
					buf[UDP_DATA_P] = 0xAA;

					if (udp_data->cmd == 12)
					{
						for ( i = 0; i < 4; i++ )
						buf[UDP_DATA_P + i + 1] = myip[i];

						send_udp_transmit(buf,5);
					}

					else if (udp_data->cmd == 4 )
					{
						uint8_t ip_check = 0;

						for ( i = 0; i < 4; i++ )
						{
							if ( udp_data->data[5 + i] != myip[i] )
							ip_check = 1;
						}

						if ( ip_check == 0 )
						{
							if ( strncmp(password,(void *)udp_data->data,5) == 0 )
							{
								eeprom_write_block ((void *)udp_data->data+9, (void *)ee_ip_addr, 4);
								buf[UDP_DATA_P + 1] = 0x01;
							}
							else
							buf[UDP_DATA_P + 1] = 0x02;

							send_udp_transmit(buf,2);

							if ( strncmp(password,(void *)udp_data->data,5) == 0 )
							RESET();
						}
					}

				}
*/				

	                        goto UDP;
			}
			else
			{
				if (strncmp("/ ",(char *)&(buf[dat_p+4]),2)==0)
				{
					plen=http200ok();
					plen=fill_tcp_data_p(buf,plen,PSTR("Try http://ip/pwd"));
					goto SENDTCP;
				}

				cmd=analyse_get_url((char *)&(buf[dat_p+4]));
	
				if (cmd==-1)
				{
					//plen=fill_tcp_data_p(buf,0,PSTR("HTTP/1.0 401 Unauthorized\r\nContent-Type: text/html\r\n\r\n<h1>401 Unauthorized</h1>"));
					plen=fill_tcp_data_p(buf,0,PSTR("HTTP/1.0 401 Unauthorized\r\n\r\nUnauthorized"));
					goto SENDTCP;
				}
			}
		}

		// Флаг изменения типа порта
		uint8_t ch_port_flag = 0;
		// Режим работы
		uint8_t mode = 0;
		uint8_t start_loop = 0;
		uint8_t end_loop = 0;
		// Команда
		char srv_cmd[33]; // = {'\0',} ;
		srv_cmd[0] = '\0';
		// Для совместимости со старыми компиляторами
		//memset(srv_cmd, 0, 33);
		if (find_key_val((char *)&(buf[dat_p+4]),srv_cmd,33,"cmd"))
		{
			if ( strcmp(srv_cmd, "get") == 0 )
			mode = 1;
			if ( strlen(srv_cmd) > 0 )
			port_execute(srv_cmd, 100, 0, 3);
		}

		/*
		if ( srv_loop_timer == 2 && tcp_client_state > 0 && tcp_client_state < 5 )
		{
			srv_loop_timer = 100;
			plen_start++;
			if ( plen_start == 30 )
			tcp_client_state = 0;
			goto UDP;
		}
		*/

		if ( find_key_val((char *)&(buf[dat_p+4]),gStrbuf,3,"pt") || strcmp(srv_cmd, "all") == 0 || srv_loop_timer == 2 )
		{
			int n = atoi(gStrbuf);

			// Опрос состояния всех портов
			if ( strcmp(srv_cmd, "all") == 0 || srv_loop_timer == 2 )
			{
				if ( strcmp(srv_cmd, "all") == 0 && srv_loop_timer < 200 && srv_loop_timer > 0 )
				srv_loop_timer = 200;

				end_loop = IO_SIZE;
				mode = 1;
			}
			else
			{
				//start_loop = atoi(gStrbuf);
				start_loop = n;
				end_loop = start_loop + 1;

				if (find_key_val((char *)&(buf[dat_p+4]),temp,25,"cnt"))
				port_cnt[start_loop] = atoi(temp);

				// Изменение значения порога для DS18B20 или АЦП без перезагрузки устройства
				if (find_key_val((char *)&(buf[dat_p+4]),temp,25,"misc"))
				goto DS18B20VAL;
			}


			/*
			if (find_key_val((char *)&(buf[dat_p+4]),temp,25,"irp0"))
			irp[0] = atoi(temp);
			if (find_key_val((char *)&(buf[dat_p+4]),temp,25,"irp1"))
			irp[1] = atoi(temp);
			if (find_key_val((char *)&(buf[dat_p+4]),temp,25,"irp2"))
			irp[2] = atoi(temp);
			if (find_key_val((char *)&(buf[dat_p+4]),temp,25,"irp3"))
			irp[3] = atoi(temp);
			if (find_key_val((char *)&(buf[dat_p+4]),temp,25,"irp4"))
			irp[4] = atoi(temp);
			*/

			//itoa(irp_long, temp, 10);

			// =======================
			/*
			if (find_key_val((char *)&(buf[dat_p+4]),srv_cmd,33,"ir"))
			{

				uint16_t irp[5];
				char ir_par[4];

				for ( i = 0; i < 5; i++ )
				{
					strcpy(ir_par, "irp");
					itoa(i, temp, 10);
					strcat(ir_par, temp);
					if (find_key_val((char *)&(buf[dat_p+4]),temp,25, ir_par))
					irp[i] = atoi(temp) / 20;
				}


				static unsigned char mask[] = {8, 4, 2, 1};
				unsigned int k, v;

				cli();

				// Инициализация PWM CTC Mode 38Khz
				//if ( n == 12 )
				mode = COM0B0;
				//else
				//mode = COM0A0;
				TCCR0A = (1<<mode) | (1<<WGM01);
				TCCR0B = 1;
				OCR0A = 163;

				// Преамбула
				//_delay_us(3450);
				delay_us(irp[0]);
				TCCR0A = 0;
				//_delay_us(1740);
				delay_us(irp[1]);

				// Процедура передачи данных
				for ( k = 0; k < strlen(srv_cmd); k++ )
				{
					v = srv_cmd[k];
					if (srv_cmd[k] >= 'A')
					v = v + 9;
					for ( i = 0; i < 4; i++ )
					{
						TCCR0A = (1<<mode) | (1<<WGM01);
	               				//_delay_us(400);
						delay_us(irp[2]);
						TCCR0A = 0;

						// Передача отдельных бит данных
						if ( srv_cmd[k] == 'i' )
						{
							delay_us(irp[3]);
							break;
						}
						else if ( srv_cmd[k] == 'o' )
						{
							delay_us(irp[4]);
							break;
						}

						if ( (v & mask[i]) != 0)
						//_delay_us(470);
						delay_us(irp[3]);
						else
						//_delay_us(1300);
						delay_us(irp[4]);
					}
				}

				TCCR0A = (1<<mode) | (1<<WGM01);
				delay_us(irp[2]);
				//_delay_us(400);
				TCCR0A = 0;

				sei();
				mode = 0;
				goto DONE;
				//memset(temp, 0, sizeof(temp));
				//srv_cmd[0]= '\0';

			}
			*/
			// =======================

			port_letter = aio[n][0];
			port_num = atoi(&aio[n][1]);

			if (find_key_val((char *)&(buf[dat_p+4]),temp,4,"dir"))
			{
				//port_letter = aio[n][0];
				//port_num = atoi(&aio[n][1]);
				//sw_port(port_letter);
				/*
				if ( temp[0] == '1' )
				*w_ddr |= (1<<port_num);
				else
				*w_ddr &= ~(1<<port_num);
				*/
				sw_dir(temp[0], port_letter, port_num);
				//goto DONE;
			}

			if (find_key_val((char *)&(buf[dat_p+4]),temp,4,"i2c"))
			{
				uint8_t i2c_data;
				int j;
				i2c_data = atoi(temp);
				char scl[12];

				find_key_val((char *)&(buf[dat_p+4]),scl,11,"scl");

				for ( j = 7; j >= 0; j-- )
				{
					//strcpy(srv_cmd, "9:");
					strcpy(srv_cmd, gStrbuf);
					strcat(srv_cmd, ":");

					if ( (1 << j) & i2c_data )
					strcat(srv_cmd, "1;");
					else
					strcat(srv_cmd, "0;");

					//strcat(srv_cmd, ";8:1;8:0;");
					strcat(srv_cmd, scl);
					strcat(srv_cmd, gStrbuf);
					strcat(srv_cmd, ":0");

					port_execute(srv_cmd, 100, 0, 3);

				}

				//*w_ddr &= ~(1<<port_num);
				sw_dir('0', port_letter, port_num);
				//strcpy(srv_cmd, "8:1;8:0");
				//strcpy(srv_cmd, scl);
				//port_execute(srv_cmd, 100, 0, 3);
				port_execute(scl, 100, 0, 3);
				sw_dir('1', port_letter, port_num);
				//*w_ddr |= (1<<port_num);
			}


			// Обработка для PWM
			if (find_key_val((char *)&(buf[dat_p+4]),temp,4,"pwm"))
			{
				//port_letter = aio[n][0];
				//port_num = atoi(&aio[n][1]);

				plen_start = atoi(temp);
				if ( plen_start > 255 )
				plen_start = 255;

				if ( port_letter == 'D' )
				{
					//_port_misc[atoi(gStrbuf)] = eeprom_read_word((unsigned int*)&ee_port_misc[atoi(gStrbuf)]);
					_port_misc[n] = eeprom_read_word((unsigned int*)&ee_port_misc[n]);
					if ( _port_misc[n] > 0 )
					{
						if ( port_cnt[n] > 0 )
						_port_misc[n] = port_cnt[n];
						else
						port_cnt[n] = _port_misc[n];

						//port_misc_cur[n] = atoi(temp);
						port_misc_cur[n] = plen_start;

					}
					else
					{
						/*
						if ( port_num == 6 )
						OCR0A = plen_start;
						else if ( port_num == 5 )
						OCR0B = plen_start;
						else if ( port_num == 3 )
						OCR2B = plen_start;
						*/
						sw_ocr(port_num);
						*w_ocr = plen_start;

					}

				}
			}

			//eeprom_read_block (_port_type, &ee_port_type, IO_SIZE);

		        plen=http200ok();
			plen_start = plen;

			//plen=fill_tcp_data(buf,plen,temp);

			if ( srv_loop_timer == 2 )
			{
				plen=fill_tcp_data(buf,plen,_spt);

				plen=fill_tcp_data_p(buf,plen,PSTR("?"));
				if ( _mdid[0] != (char)255 )
				{
					plen=fill_tcp_data_p(buf,plen,PSTR("mdid="));
					plen=fill_tcp_data(buf,plen,_mdid);
				}
				plen=fill_tcp_data_p(buf,plen,PSTR("&all="));
			}


			for ( i = start_loop; i < end_loop; i++ )
			{

				port_letter = aio[i][0];
				port_num = atoi(&aio[i][1]);
				sw_port(port_letter);

				if ( mode == 0 )
				{

					//plen=fill_tcp_data_p(buf,plen,PSTR("<a href=/"));
					plen=fill_tcp_data_p(buf,plen,html_ahref);
					plen=fill_tcp_data(buf,plen,password);
					plen=fill_tcp_data_p(buf,plen,PSTR(">Back</a><br>"));
	
					if ( port_letter == 'A' )
					{
						plen=fill_tcp_data_p(buf,plen,PSTR("A"));
						plen=fill_tcp_data(buf,plen,&aio[i][1]);
					}
					else
					{
						plen=fill_tcp_data_p(buf,plen,PSTR("P"));
						//itoa(n, gStrbuf, 10);
						plen=fill_tcp_data(buf,plen,gStrbuf);
					}
				}

				// Если порт уже сконфигурирован
				//if ( port_num < IO_SIZE && ( _port_type[i] != 255 || ( port_letter == 'A' && ( port_num == 6 || port_num == 7 ) ) ) )
				if ( port_num < IO_SIZE || ( port_letter == 'A' && ( port_num == 6 || port_num == 7 ) ) )
				{

					// Если порт является АЦП
					if ( _port_type[i] == 2 || port_letter == 'A' )
					{
						if ( mode == 0 )
						plen=fill_tcp_data_p(buf,plen,PSTR("/"));

						//if ( port_letter == 'A' )
						//ADMUX = (1 << REFS0) + 6; 
						//else
						ADMUX = (1 << REFS0) + port_num; 
						//ADMUX |= (1 << ADLAR); 
						ADCSRA|=(1<<ADSC);
						while (bit_is_set(ADCSRA, ADSC));
						//_delay_loop_1(0); // 60us
						//_delay_loop_1(0); // 60us
						uint16_t my_val;
						uint8_t my_val_low;
						my_val_low = ADCL;
						my_val = (ADCH<<8)|my_val_low;
						itoa (my_val, temp, 10);
						plen=fill_tcp_data(buf,plen, temp);
					}
					// Если порт является выходом
					//else if ( bit_is_set(avr_port, port_num) && _port_type[i] == 1 )
					else if ( bit_is_set(*w_ddr, port_num) && _port_type[i] == 1 )
					{
						// Форма для PWM
						if (_port_m[i] == 1 )
						{

							if ( mode == 0 )
							{
								plen=fill_tcp_data_p(buf,plen,PSTR("<br><form style=display:inline action=/"));
								plen=fill_tcp_data(buf,plen,password);
								plen=fill_tcp_data_p(buf,plen,PSTR("/>"));
								//plen=fill_tcp_data_p(buf,plen,PSTR("<input type=hidden name=pt value="));
								plen=fill_tcp_data_p(buf,plen,html_input_hidden);
								plen=fill_tcp_data_p(buf,plen,PSTR("pt value="));
								plen=fill_tcp_data(buf,plen,gStrbuf);
								//plen=fill_tcp_data_p(buf,plen,PSTR(">"));

								//plen=fill_tcp_data_p(buf,plen,PSTR(">PWM (0-255): <input name=pwm size=3 value="));
								plen=fill_tcp_data_p(buf,plen,PSTR(">PWM <input name=pwm size=3 value="));
							}

							if ( port_letter == 'D' )
							{
								if ( _port_misc[i] > 0 && mode == 0 )
								itoa(port_misc_cur[i], temp, 10);
								else
								{
									if ( port_num == 5 )
									itoa(OCR0B, temp, 10);
									else if ( port_num == 6 )
									itoa(OCR0A, temp, 10);
									else if ( port_num == 3 )
									itoa(OCR2B, temp, 10);
								}
							}
							//itoa(port_cnt[i], temp, 10);
							plen=fill_tcp_data(buf,plen,temp);
						
							if ( mode == 0 )
							plen=fill_tcp_data_p(buf,plen,PSTR("> <input type=submit value=change></form>"));

						}
						// Форма для SWITCH
						else
						{
							if ( mode == 0 )
							plen=fill_tcp_data_p(buf,plen,PSTR("/"));
		
							//if ( bit_is_set(avr_port_port, port_num) )
							if ( bit_is_set(*w_port, port_num) )
							plen=fill_tcp_data_p(buf,plen,html_on);
							else
							plen=fill_tcp_data_p(buf,plen,html_off);

							if ( mode == 0 )
							{
								plen=fill_tcp_data_p(buf,plen,html_br);
								plen=fill_tcp_data_p(buf,plen,html_ahref);
								plen=fill_tcp_data(buf,plen,password);
								//plen=fill_tcp_data_p(buf,plen,PSTR("/?pt="));
								plen=fill_tcp_data_p(buf,plen,html_pt);
								plen=fill_tcp_data(buf,plen,gStrbuf);
								//plen=fill_tcp_data_p(buf,plen,PSTR("&cmd="));
								plen=fill_tcp_data_p(buf,plen,html_cmd);
								plen=fill_tcp_data(buf,plen,gStrbuf);
								plen=fill_tcp_data_p(buf,plen,PSTR(":1><h1>ON</h1></a> <a href=/"));

								//plen=fill_tcp_data_p(buf,plen,PSTR("<a href=/"));
								//plen=fill_tcp_data_p(buf,plen,html_ahref);
								plen=fill_tcp_data(buf,plen,password);
								//plen=fill_tcp_data_p(buf,plen,PSTR("/?pt="));
								plen=fill_tcp_data_p(buf,plen,html_pt);
								plen=fill_tcp_data(buf,plen,gStrbuf);
								//plen=fill_tcp_data_p(buf,plen,PSTR("&cmd="));
								plen=fill_tcp_data_p(buf,plen,html_cmd);
								plen=fill_tcp_data(buf,plen,gStrbuf);
								plen=fill_tcp_data_p(buf,plen,PSTR(":0><h1>OFF</h1></a><br>"));

								//plen=fill_tcp_data_p(buf,plen,html_br);
							}
						}
					}
					else if ( _port_type[i] == 3 )
					{
						if ( _port_d[i] != 4)
						{
							dht11_temperature = 0;
							dht11_humidity = 0;
							dht_type = _port_d[i];
							//int d = 0;

							if ( mode == 0 )
							{
								//plen=fill_tcp_data_p(buf,plen,html_br);
								plen=fill_tcp_data_p(buf,plen,PSTR("<br>t:"));
							}


							if ( dht_type == 3 )
							{
								if ( port_misc_cur[i] == 13000 )
								strcpy(temp, "NA");
								else
								{
									if ( port_misc_cur[i] < 0 )
									sprintf(temp, "-%d.%02d", abs(port_misc_cur[i]/100), abs(port_misc_cur[i] % 100));
									else
									sprintf(temp, "%d.%02d", port_misc_cur[i]/100, port_misc_cur[i] % 100);
								}
								plen=fill_tcp_data(buf,plen,temp);
							}
							else
							{
								if ( srv_loop_timer > 11900 )
								_delay_ms(150);
								dht_gettemperaturehumidity(&dht11_temperature, &dht11_humidity, _port_d[i]);

								sprintf(temp, "%d.%02d", dht11_temperature/100, abs(dht11_temperature % 100));
								plen=fill_tcp_data(buf,plen,temp);
								sprintf(temp, "%d.%02d", dht11_humidity/100, abs(dht11_humidity % 100));
								if ( mode == 0 )
								plen=fill_tcp_data_p(buf,plen,html_br);
								else
								plen=fill_tcp_data_p(buf,plen,PSTR("/"));
								if ( mode == 0 )
								plen=fill_tcp_data_p(buf,plen,PSTR("h:"));
								plen=fill_tcp_data(buf,plen,temp);
							}       
						}
					}
					// Если порт является входом
					else
					{
						if ( mode == 0 )
						plen=fill_tcp_data_p(buf,plen,PSTR("/"));

						//if ( bit_is_clear(avr_port_pin, port_num) )
						if ( bit_is_clear(*w_pin, port_num) )
						plen=fill_tcp_data_p(buf,plen,html_on);
						else
						plen=fill_tcp_data_p(buf,plen,html_off);
	
						if ( _port_type[i] == 0 )
						{
							plen=fill_tcp_data_p(buf,plen,PSTR("/"));
							utoa(port_cnt[i], temp, 10);
							plen=fill_tcp_data(buf,plen,temp);
						}
					}
				}

				if ( mode == 0)
				{
					//plen=fill_tcp_data_p(buf,plen,PSTR("<form action=/"));
					plen=fill_tcp_data_p(buf,plen,html_form);
					plen=fill_tcp_data(buf,plen,password);
					plen=fill_tcp_data_p(buf,plen,PSTR("/>"));
					plen=fill_tcp_data_p(buf,plen,html_input_hidden);
					//plen=fill_tcp_data_p(buf,plen,PSTR("<input type=hidden name=pn value="));
					plen=fill_tcp_data_p(buf,plen,PSTR("pn value="));
					plen=fill_tcp_data(buf,plen,gStrbuf);
					//plen=fill_tcp_data_p(buf,plen,PSTR(">"));
					plen=fill_tcp_data_p(buf,plen,html_cl);

					//if ( port_letter != 'A' && _preset != 1 )
					if ( port_letter != 'A' )
					{
						plen=fill_tcp_data_p(buf,plen,PSTR("Type <select name=pty><option value=255"));

						if ( _port_type[i] == 255 )
						plen=fill_tcp_data_p(buf,plen,selected);
						// NO CLOSING TAGS!
						//plen=fill_tcp_data_p(buf,plen,PSTR(">NC</option><option value=0"));
						plen=fill_tcp_data_p(buf,plen,PSTR(">NC<option value=0"));
						//if ( bit_is_clear(avr_port, port_num) && _port_type[i] == 0 )
						if ( bit_is_clear(*w_ddr, port_num) && _port_type[i] == 0 )
						plen=fill_tcp_data_p(buf,plen,selected);
						// NO CLOSING TAGS!
						//plen=fill_tcp_data_p(buf,plen,PSTR(">In</option><option value=1"));
						plen=fill_tcp_data_p(buf,plen,PSTR(">In<option value=1"));
						//if ( bit_is_set(avr_port, port_num) && _port_type[i] == 1 )
						if ( bit_is_set(*w_ddr, port_num) && _port_type[i] == 1 )
						plen=fill_tcp_data_p(buf,plen,selected);
						// NO CLOSING TAGS!
						//plen=fill_tcp_data_p(buf,plen,PSTR(">Out</option><option value=3"));
						plen=fill_tcp_data_p(buf,plen,PSTR(">Out<option value=3"));
						if ( _port_type[i] == 3 )
						plen=fill_tcp_data_p(buf,plen,selected);
						// NO CLOSING TAGS!
						//plen=fill_tcp_data_p(buf,plen,PSTR(">DSen</option>"));
						plen=fill_tcp_data_p(buf,plen,PSTR(">DSen"));

						if ( port_letter == 'C' )
						{
							plen=fill_tcp_data_p(buf,plen,PSTR("<option value=2"));
							if ( _port_type[i] == 2 )
							plen=fill_tcp_data_p(buf,plen,selected);
							// NO CLOSING TAGS!
							//plen=fill_tcp_data_p(buf,plen,PSTR(">ADC</option>"));
							plen=fill_tcp_data_p(buf,plen,PSTR(">ADC"));
						}

						plen=fill_tcp_data_p(buf,plen,PSTR("</select>"));


					}
					//else if ( port_letter != 'A' && _preset == 1 )
					/*
					else if ( port_letter != 'A' )
					{
						plen=fill_tcp_data_p(buf,plen,PSTR("Type "));
						//if ( bit_is_clear(avr_port, port_num) && _port_type[i] == 0 )
						if ( bit_is_clear(*w_ddr, port_num) && _port_type[i] == 0 )
						plen=fill_tcp_data_p(buf,plen,html_in);
						//else if ( bit_is_set(avr_port, port_num) && _port_type[i] == 1 )
						else if ( bit_is_set(*w_ddr, port_num) && _port_type[i] == 1 )
						plen=fill_tcp_data_p(buf,plen,html_out);
						else
						plen=fill_tcp_data_p(buf,plen,PSTR("NC"));
					}*/

					//if ( _port_type[i] == 2 || port_letter == 'A' )
					if ( _port_type[i] == 2 || ( _port_type[i] == 3 && _port_d[i] == 3 ) || port_letter == 'A' )
					{
						//if ( _port_type[i] == 2 )
						if ( port_letter != 'A' )
						plen=fill_tcp_data_p(buf,plen,html_br);

						plen=fill_tcp_data_p(buf,plen,PSTR("Mode <select name=m><option value=0"));
						if ( _port_m[i] == 0 || _port_m[i] == 255 )
						plen=fill_tcp_data_p(buf,plen,selected);
						// NO CLOSING TAGS!
						//plen=fill_tcp_data_p(buf,plen,PSTR(">Norm</option><option value=1"));
						plen=fill_tcp_data_p(buf,plen,PSTR(">N<option value=1"));
						if (_port_m[i] == 1 )
						plen=fill_tcp_data_p(buf,plen,selected);
						// NO CLOSING TAGS!
						//plen=fill_tcp_data_p(buf,plen,PSTR(">></option><option value=2"));
						plen=fill_tcp_data_p(buf,plen,PSTR(">><option value=2"));
						if (_port_m[i] == 2 )
						plen=fill_tcp_data_p(buf,plen,selected);
						// NO CLOSING TAGS!
						//plen=fill_tcp_data_p(buf,plen,PSTR("><</option><option value=3"));
						plen=fill_tcp_data_p(buf,plen,PSTR("><<option value=3"));
						if (_port_m[i] == 3 )
						plen=fill_tcp_data_p(buf,plen,selected);
						// maxlength 4
						// NO CLOSING TAGS!
						//plen=fill_tcp_data_p(buf,plen,PSTR("><></option></select><br>Val <input name=misc size=4 value="));
						plen=fill_tcp_data_p(buf,plen,PSTR("><></select><br>Val <input name=misc value="));

						if ( _port_type[i] == 3 )
						sprintf(temp,"%d.%02d",_port_misc[i]/100, abs(_port_misc[i] % 100));
						else
						{
							if ( _port_misc[i] < 0 )
							temp[0] = '\0';
							else
							sprintf(temp,"%u",_port_misc[i]);
						}

						plen=fill_tcp_data(buf,plen,temp);
						//plen=fill_tcp_data_p(buf,plen,html_cl);

						// Hyst
						plen=fill_tcp_data_p(buf,plen,PSTR("><br>Hst <input name=hst value="));
						if ( _port_type[i] == 3 )
						sprintf(temp,"%d.%02d",port_cnt[i]/100, abs(port_cnt[i] % 100));
						else
						utoa(port_cnt[i], temp, 10);
						plen=fill_tcp_data(buf,plen,temp);
						plen=fill_tcp_data_p(buf,plen,html_cl);


					}

					//if ( ( bit_is_clear(avr_port, port_num) && _port_type[i] == 0 ) || _port_type[i] == 2 || port_letter == 'A' )
					if ( ( bit_is_clear(*w_ddr, port_num) && _port_type[i] == 0 ) || _port_type[i] == 2 || ( _port_type[i] == 3 && _port_d[i] == 3 ) || port_letter == 'A' )
					{
						// maxlength 23
						plen=fill_tcp_data_p(buf,plen,PSTR("<br>Act <input name=ecmd value=\""));
						eeprom_read_block (temp, &ee_cmd[i], 23);
						//if ( temp[0] != (char)255 && strlen(temp) > 0 ) 
						if ( *temp != (char)0xff && *temp )
						plen=fill_tcp_data(buf,plen, temp);
						//plen=fill_tcp_data_p(buf,plen,PSTR("\"><br>"));
						//plen=fill_tcp_data_p(buf,plen,html_scbr);

						// maxlength 29
						//plen=fill_tcp_data_p(buf,plen,PSTR("Net <input size=30 name=eth value=\""));
						plen=fill_tcp_data_p(buf,plen,PSTR("\"><br>Net <input name=eth value=\""));

						eeprom_read_block (_eth_addr, &ee_eth_cmd[i], 4);
						if ( _eth_addr[0] != 255 )
						{
							snprintf_P(temp,sizeof(temp),PSTR("%d.%d.%d.%d"),(int)_eth_addr[0],(int)_eth_addr[1],(int)_eth_addr[2],(int)_eth_addr[3]);
							plen=fill_tcp_data(buf,plen, temp);

							memset(temp, 0, sizeof(temp));

							if ( eeprom_read_byte ((void*)ee_eth_cmd[i] + 4) == ':' )
							{
								snprintf_P(temp,sizeof(temp),PSTR(":%u"),eeprom_read_word ((void*)ee_eth_cmd[i] + 5));
								plen=fill_tcp_data(buf,plen,temp);
								//memset(temp, 0, sizeof(temp));
								eeprom_read_block (temp, (void*)ee_eth_cmd[i] + 7, 10);
							}
							else
							eeprom_read_block (temp, (void*)ee_eth_cmd[i] + 4, 13);

							plen=fill_tcp_data_p(buf,plen,PSTR("/"));
							plen=fill_tcp_data(buf,plen, temp);
						}

						//plen=fill_tcp_data_p(buf,plen,PSTR("\">"));

						plen=fill_tcp_data_p(buf,plen,PSTR("\"> <input type=checkbox name=naf value=1"));
						if ( eeprom_read_byte(&ee_netact_flag[i]) == 1 )
						//plen=fill_tcp_data_p(buf,plen,PSTR(" checked"));
						//plen=fill_tcp_data_p(buf,plen,PSTR(">"));
						plen=fill_tcp_data_p(buf,plen,html_checked);
						plen=fill_tcp_data_p(buf,plen,html_cl);

					}

					//if ( bit_is_clear(avr_port, port_num) && _port_type[i] == 0 )
					if ( bit_is_clear(*w_ddr, port_num) && _port_type[i] == 0 )
					{
						plen=fill_tcp_data_p(buf,plen,PSTR("<br>Mode <select name=m><option value=0"));
						if ( _port_m[i] == 0 || _port_m[i] == 255 )
						plen=fill_tcp_data_p(buf,plen,selected);
						// NO CLOSING TAGS!
						//plen=fill_tcp_data_p(buf,plen,PSTR(">P</option><option value=1"));
						plen=fill_tcp_data_p(buf,plen,PSTR(">P<option value=1"));
						if (_port_m[i] == 1 )
						plen=fill_tcp_data_p(buf,plen,selected);
						// NO CLOSING TAGS!
						//plen=fill_tcp_data_p(buf,plen,PSTR(">P&R</option><option value=2"));
						plen=fill_tcp_data_p(buf,plen,PSTR(">P&R<option value=2"));
						if (_port_m[i] == 2 )
						plen=fill_tcp_data_p(buf,plen,selected);
						// NO CLOSING TAGS!
						//plen=fill_tcp_data_p(buf,plen,PSTR(">R</option></select>"));
						plen=fill_tcp_data_p(buf,plen,PSTR(">R</select> <input type=checkbox name=misc value=1"));

						//plen=fill_tcp_data_p(buf,plen,PSTR(" <input type=checkbox name=misc value=1"));

						if ( eeprom_read_word((unsigned int*)&ee_port_misc[i]) == 1 )
						plen=fill_tcp_data_p(buf,plen,html_checked);
						//plen=fill_tcp_data_p(buf,plen,PSTR(" checked"));
						//plen=fill_tcp_data_p(buf,plen,PSTR(">"));
						//plen=fill_tcp_data_p(buf,plen,html_cl);
						//plen=fill_tcp_data_p(buf,plen,html_checked_cl);

						plen=fill_tcp_data_p(buf,plen,PSTR("><br>Raw <input type=checkbox name=d value=1"));
						if ( _port_d[i] == 1 || _port_d[i] == 7 )
						plen=fill_tcp_data_p(buf,plen,html_checked);
						//plen=fill_tcp_data_p(buf,plen,PSTR(">"));
						plen=fill_tcp_data_p(buf,plen,html_cl);

					}
					//else if ( bit_is_set(avr_port, port_num) && _port_type[i] == 1 )
					else if ( bit_is_set(*w_ddr, port_num) && _port_type[i] == 1 )
					{
						// Логическое состояние пина по умолчанию
						plen=fill_tcp_data_p(buf,plen,PSTR("<br>Def "));

						// Если выход - ШИМ (PWM)
						if ( _port_m[i] == 1 )
						{
							plen=fill_tcp_data_p(buf,plen,PSTR("<input name=d size=3 value="));
							itoa(_port_d[i], temp, 10);
							plen=fill_tcp_data(buf,plen,temp);
							//plen=fill_tcp_data_p(buf,plen,PSTR(">"));
							plen=fill_tcp_data_p(buf,plen,html_cl);
						}
						// Если обычный ключ (SW)
						else
						{
							plen=fill_tcp_data_p(buf,plen,PSTR("<select name=d><option value=0"));
							if (_port_d[i] != 1 )
							plen=fill_tcp_data_p(buf,plen,selected);
							// NO CLOSING TAGS!
							//plen=fill_tcp_data_p(buf,plen,PSTR(">0</option><option value=1"));
							plen=fill_tcp_data_p(buf,plen,PSTR(">0<option value=1"));
							if (_port_d[i] == 1 )
							plen=fill_tcp_data_p(buf,plen,selected);
							// NO CLOSING TAGS!
							//plen=fill_tcp_data_p(buf,plen,PSTR(">1</option></select>"));
							plen=fill_tcp_data_p(buf,plen,PSTR(">1</select>"));
						}

						// Тип выхода: Ключ/ШИМ
						//if ( _preset != 1 && port_letter == 'D' && ( port_num == 3 || port_num == 5 || port_num == 6 ) )
						if ( port_letter == 'D' && ( port_num == 3 || port_num == 5 || port_num == 6 ) )
						{
							plen=fill_tcp_data_p(buf,plen,PSTR("<br>Mode <select name=m><option value=0"));
							if ( _port_m[i] != 1 )
							plen=fill_tcp_data_p(buf,plen,selected);
							// NO CLOSING TAGS!
							//plen=fill_tcp_data_p(buf,plen,PSTR(">SW</option><option value=1"));
							plen=fill_tcp_data_p(buf,plen,PSTR(">SW<option value=1"));
							if (_port_m[i] == 1 )
							plen=fill_tcp_data_p(buf,plen,selected);
							// NO CLOSING TAGS!
							//plen=fill_tcp_data_p(buf,plen,PSTR(">PWM</option></select>"));
							plen=fill_tcp_data_p(buf,plen,PSTR(">PWM</select>"));
						}

						if ( _port_m[i] == 1 )
						{
							//plen=fill_tcp_data_p(buf,plen,PSTR("<br>Smooth <input type=checkbox name=misc value=1"));
							plen=fill_tcp_data_p(buf,plen,PSTR("<br>Smooth <input type=checkbox name=misc value=1"));
							if ( eeprom_read_word((unsigned int*)&ee_port_misc[i]) > 0 )
							{
								//plen=fill_tcp_data_p(buf,plen,html_checked);
								//plen=fill_tcp_data_p(buf,plen,PSTR(">"));
								//plen=fill_tcp_data_p(buf,plen,html_checked_cl);
								plen=fill_tcp_data_p(buf,plen,PSTR(" checked> <input name=m2 size=3 value="));
								itoa(eeprom_read_word((unsigned int*)&ee_port_misc[i]), temp, 10);
								plen=fill_tcp_data(buf,plen,temp);
							}
							//plen=fill_tcp_data_p(buf,plen,PSTR(">"));
							plen=fill_tcp_data_p(buf,plen,html_cl);

						}

					}
					else if ( _port_type[i] == 3 )
					{
						plen=fill_tcp_data_p(buf,plen,PSTR("<br>Sen <select name=d><option value=2"));
						if (_port_d[i] < 3 )
						plen=fill_tcp_data_p(buf,plen,selected);
						plen=fill_tcp_data_p(buf,plen,PSTR(">DHT<option value=3"));
						if (_port_d[i] == 3 )
						plen=fill_tcp_data_p(buf,plen,selected);
						plen=fill_tcp_data_p(buf,plen,PSTR(">1W<option value=4"));
						if (_port_d[i] == 4 )
						plen=fill_tcp_data_p(buf,plen,selected);
						plen=fill_tcp_data_p(buf,plen,PSTR(">iB</select>"));

						/*
						plen=fill_tcp_data_p(buf,plen,PSTR("<br>Sen <select name=d><option value=1"));
						if (_port_d[i] < 2 )
						plen=fill_tcp_data_p(buf,plen,selected);
						plen=fill_tcp_data_p(buf,plen,PSTR(">DHT11<option value=2"));
						if (_port_d[i] == 2 )
						plen=fill_tcp_data_p(buf,plen,selected);
						plen=fill_tcp_data_p(buf,plen,PSTR(">DHT22<option value=3"));
						if (_port_d[i] == 3 )
						plen=fill_tcp_data_p(buf,plen,selected);
						plen=fill_tcp_data_p(buf,plen,PSTR(">1W<option value=4"));
						if (_port_d[i] == 4 )
						plen=fill_tcp_data_p(buf,plen,selected);
						plen=fill_tcp_data_p(buf,plen,PSTR(">iB</select>"));
						*/

					}

					plen=fill_tcp_data_p(buf,plen,html_submit);
				}

				if ( mode == 1 && i < end_loop - 1 )
				plen=fill_tcp_data_p(buf,plen,PSTR(";"));
			}

			if ( srv_loop_timer == 2 )
			{
				plen=fill_tcp_data_p(buf,plen,PSTR("*"));
				srv_loop_timer = 1;

			
				goto UDP;
			}

		}
		else if (find_key_val((char *)&(buf[dat_p+4]),gStrbuf,25,"pn"))
		{
			memset(temp, 0, sizeof(temp));
			int n = atoi(gStrbuf);

			if (find_key_val((char *)&(buf[dat_p+4]),temp,25,"pty"))
			{
				// Произошло изменение типа порт. Сбрасываем значение режима порта
				if ( _port_type[n] != atoi(temp) )
				{
					ch_port_flag = 1;
					_port_m[n] = 255;
					eeprom_write_block (&_port_m[n], &ee_port_m[n], 1);
				}

				if ( strcmp(temp,"NC") )
				{
					_port_type[n] = atoi(temp);
					eeprom_write_block (&_port_type[n], &ee_port_type[n], 1);
					reset_flag = 1;
				}
			}

			// Обрабатываем параметры только в том случае, если нет изменения типа порта
			if ( ch_port_flag == 0 )
			{
				if (find_key_val2((char *)&(buf[dat_p+4]),temp,42,"ecmd"))
				{
					urldecode(temp);
					eeprom_write_block (&temp, &ee_cmd[n], 23);
					reset_flag = 1;
				}

				if (find_key_val2((char *)&(buf[dat_p+4]),temp,42,"eth"))
				{
					//if ( strlen(temp) > 0 )
					{
						urldecode(temp);

						char urlpar[13];
						uint8_t urlpar_flag = 0;
						uint8_t urlpar_flag2 = 0;

						if ( temp[0] == '\0' )
						_eth_addr[0] = 255;
						else
						{
							memset(urlpar, 0, sizeof(urlpar));

							for ( i = 0; i < strlen(temp); i++ )
							{
								if ( urlpar_flag == 0 && temp[i] == ':' )
								{
									eeprom_write_byte ((void*)ee_eth_cmd[n]+4, ':');
									eeprom_write_word((void*)ee_eth_cmd[n]+5, strtoul(&temp[i+1], NULL, 10));
									urlpar_flag2 = 3;

								}
								if ( urlpar_flag > 0 )
								{
									urlpar[urlpar_flag-1] = temp[i];
									urlpar_flag++;
								}
								else
								{
									if ( temp[i] == '/' )
									urlpar_flag++;
								}
							}

							decode_ip(temp, _eth_addr);
						}

						eeprom_write_block (_eth_addr, &ee_eth_cmd[n], 4);

						eeprom_write_block (urlpar, (void*)ee_eth_cmd[n] + 4 + urlpar_flag2, 13 - urlpar_flag2);
						reset_flag = 1;
					}
				}

				if (find_key_val((char *)&(buf[dat_p+4]),temp,25,"naf"))
				eeprom_write_byte(&ee_netact_flag[n], 1);
				else
				eeprom_write_byte(&ee_netact_flag[n], 0);

				if (find_key_val((char *)&(buf[dat_p+4]),temp,25,"m"))
				{
					_port_m[n] = atoi(temp);
					eeprom_write_block (&_port_m[n], &ee_port_m[n], 1);
				}

				if (find_key_val((char *)&(buf[dat_p+4]),temp,25,"d"))
				{
					_port_d[n] = atoi(temp);
					eeprom_write_block (&_port_d[n], &ee_port_d[n], 1);
				}
				else
				eeprom_write_block (0, &ee_port_d[n], 1);


				if (find_key_val((char *)&(buf[dat_p+4]),temp,8,"hst"))
				{
					char dec = 0;
					if ( _port_type[n] == 3 )
					{
						char* c;
						if ( (c = strchr(temp, '.')) != NULL)
						{
							c[3] = '\0';
							dec = atoi(c+1);
							if ( strlen(c) == 2 )
							dec = dec * 10;
							//if ( dec > 100 )
							//dec = dec / 100;
						}
						port_cnt[n] = atoi(temp) * 100 + dec;
					}
					else
					port_cnt[n] = atoi(temp);

					eeprom_write_word(&ee_port_hyst[n], port_cnt[n]);
				}


				DS18B20VAL:
				if (find_key_val((char *)&(buf[dat_p+4]),temp,25,"misc"))
				{
					n = atoi(gStrbuf);

					if (find_key_val((char *)&(buf[dat_p+4]),temp,25,"m2"))
					_port_misc[n] = atoi(temp);
					else
					{
						//if ( _port_type[atoi(gStrbuf)] == 3 )
						if ( _port_type[n] == 3 )
						{
							char dec = 0;
							char* c;
							if ( (c = strchr(temp, '.')) != NULL)
							{
								c[3] = '\0';
								dec = atoi(c+1);
								if ( strlen(c) == 2 )
								dec = dec * 10;
								//if ( dec > 100 )
								//dec = dec / 100;
							}
							//_port_misc[atoi(gStrbuf)] = atoi(temp) * 10 + dec;
							_port_misc[n] = atoi(temp) * 100 + dec;

							//port_misc_cur[atoi(gStrbuf)] = _port_misc[atoi(gStrbuf)];
							port_misc_cur[n] = _port_misc[n];
							adc_timer = 840;
						}
						else
						//_port_misc[atoi(gStrbuf)] = atoi(temp);
						_port_misc[n] = atoi(temp);
					}
				}
				else
				_port_misc[atoi(gStrbuf)] = 0;

				eeprom_write_block (&_port_misc[atoi(gStrbuf)], &ee_port_misc[atoi(gStrbuf)], 2);

			}


		        plen=http200ok();
			plen=fill_tcp_data_p(buf,plen,html_ahref);
			plen=fill_tcp_data(buf,plen,password);
			plen=fill_tcp_data_p(buf,plen,PSTR(">Back</a><br>Restarted"));

		}
		/*
		else if (find_key_val((char *)&(buf[dat_p+4]),gStrbuf,5,"tget"))
		{
		        plen=http200ok();
			sprintf(temp,"%d", readchiptemp());
			plen=fill_tcp_data(buf,plen,temp);
		}
		*/
		else if (find_key_val((char *)&(buf[dat_p+4]),gStrbuf,2,"cf"))
		{
			/*Temp-check*/
			//uint8_t temp_check_flag2 = 0;
			uint8_t cf_mode = atoi(gStrbuf);


			/*Temp-check*/
			//if (find_key_val((char *)&(buf[dat_p+4]),gStrbuf,25,"tc"))
			//temp_check_flag2 = 1;

			if (find_key_val((char *)&(buf[dat_p+4]),gStrbuf,25,"eip"))
			{
				decode_ip(gStrbuf, _ip_addr);
				eeprom_write_block (_ip_addr, (void *) ee_ip_addr, 4);
				reset_flag = 1;

				/*Temp-check*/
				//eeprom_write_byte((uint8_t*)&ee_temp_check, temp_check_flag2);
			}

			if (find_key_val2((char *)&(buf[dat_p+4]),gStrbuf,25,"sip"))
			{
				if ( !strcmp(gStrbuf,"") )
				{
					_sip_addr[0] = 255;
					_sip_addr[1] = 255;
					_sip_addr[2] = 255;
					_sip_addr[3] = 255;
					eeprom_write_word(&ee_srv_port, 80);
				}
				else
				decode_ip(gStrbuf, _sip_addr);
				eeprom_write_block (_sip_addr, &ee_sip_addr, 4);

				for ( i = 0; i < strlen(gStrbuf); i++ )
				{
					if ( gStrbuf[i] == 'A' || gStrbuf[i] == ':' )
					{
						_srv_port = strtoul(&gStrbuf[i+1], NULL, 10);
						eeprom_write_word(&ee_srv_port, _srv_port);
						break;
					}
				}

				reset_flag = 1;
			}

			// Gateway IP
			if (find_key_val2((char *)&(buf[dat_p+4]),gStrbuf,25,"gw"))
			{
				if ( !strcmp(gStrbuf,"") )
				{
					_sip_addr[0] = 255;
					_sip_addr[1] = 255;
					_sip_addr[2] = 255;
					_sip_addr[3] = 255;
				}
				else
				decode_ip(gStrbuf, _sip_addr);
				eeprom_write_block (_sip_addr, &ee_gw_addr, 4);
				reset_flag = 1;
			}


			if (find_key_val2((char *)&(buf[dat_p+4]),gStrbuf,25,"sct"))
			{
				urldecode(gStrbuf);
				eeprom_write_block (gStrbuf, &ee_spt, 15);
				reset_flag = 1;
			}

			if (find_key_val((char *)&(buf[dat_p+4]),gStrbuf,25,"pwd"))
			{
				eeprom_write_block (gStrbuf, &ee_pwd, 6);
				strcpy(password, gStrbuf);
				reset_flag = 1;
			}

			if (find_key_val2((char *)&(buf[dat_p+4]),gStrbuf,5,"at"))
			{
				_alarm_temp=atoi(gStrbuf);
				eeprom_write_byte((uint8_t*)&ee_alarm_temp, _alarm_temp);
				reset_flag = 1;
			}

			if (find_key_val2((char *)&(buf[dat_p+4]),gStrbuf,25,"mdid"))
			{
				eeprom_write_block (gStrbuf, &ee_mdid, 5);
				reset_flag = 1;

				if (find_key_val((char *)&(buf[dat_p+4]),gStrbuf,25,"sl"))
				eeprom_write_byte((uint8_t*)&ee_srv_loop, 1);
				else
				eeprom_write_byte((uint8_t*)&ee_srv_loop, 0);

			}

			/*
			if (find_key_val2((char *)&(buf[dat_p+4]),gStrbuf,5,"pr"))
			{
				if ( !strcmp(gStrbuf,"") )
				_preset = 255;
				else
				_preset=atoi(gStrbuf);
				eeprom_write_byte((uint8_t*)&ee_preset, _preset);
				reset_flag = 1;
			}
			*/

			/*
			if (find_key_val2((char *)&(buf[dat_p+4]),gStrbuf,25,"pr"))
			{
				if ( _preset != atoi(gStrbuf) )
				{
					eeprom_write_byte((uint8_t*)&ee_preset, atoi(gStrbuf));

					if ( atoi(gStrbuf) == 1 )
					{
						for ( i = 0; i < IO_SIZE; i++ )
						{
							port_letter = aio[i][0];
							//port_num = atoi(&aio[i][1]);
							if ( i < 7 )
							_port_type[i] = 0;
							else if ( port_letter == 'A' )
							_port_type[i] = 255;
							else
							_port_type[i] = 1;

							if ( port_letter != 'A' )
							_port_m[i] = 255;
						}

						eeprom_write_block (&_port_m, &ee_port_m, sizeof(_port_m));
						eeprom_write_block (&_port_type, &ee_port_type, sizeof(_port_type));
						_preset = 1;
						reset_flag = 1;
					}
					else
					_preset = 0;
				}

				reset_flag = 1;
			}
			*/

			plen=http200ok();

			eeprom_read_block ((void *)_ip_addr, (const void *)ee_ip_addr,4);
			eeprom_read_block ((void *)_sip_addr, (const void *)&ee_sip_addr,4);
			eeprom_read_block (_spt, &ee_spt,15);
			eeprom_read_block (_mdid, &ee_mdid,5);
			//eeprom_read_block (password, &ee_pwd,6);

			if ( _ip_addr[0] == 255 )
			{
				//for ( i = 0; i < 4; i++ )
				//_ip_addr[i] = myip[i];

				// Развернутый цикл занимает меньше памяти и работает быстрее...
				_ip_addr[0] = myip[0];
				_ip_addr[1] = myip[1];
				_ip_addr[2] = myip[2];
				_ip_addr[3] = myip[3];
			}

			//plen=fill_tcp_data_p(buf,plen,PSTR("<a href=/"));
			plen=fill_tcp_data_p(buf,plen,html_ahref);
			plen=fill_tcp_data(buf,plen,password);
			plen=fill_tcp_data_p(buf,plen,PSTR(">Back</a> | <a href=/"));
			plen=fill_tcp_data(buf,plen,password);
			if ( cf_mode == 1 )
			plen=fill_tcp_data_p(buf,plen,PSTR("/?cf=2>Megad-ID</a><br>"));
			else if ( cf_mode == 2 )
			plen=fill_tcp_data_p(buf,plen,PSTR("/?cf=1>Config</a><br>"));


			//plen=fill_tcp_data_p(buf,plen,PSTR("<form action=/"));
			plen=fill_tcp_data_p(buf,plen,html_form);
			plen=fill_tcp_data(buf,plen,password);
			plen=fill_tcp_data_p(buf,plen,PSTR("/>"));
			plen=fill_tcp_data_p(buf,plen,html_input_hidden);
			plen=fill_tcp_data_p(buf,plen,PSTR("cf value="));

			if ( cf_mode == 1 )
			{
				plen=fill_tcp_data_p(buf,plen,PSTR("1>"));

				snprintf_P(temp,sizeof(temp),PSTR("IP <input name=eip value=%d.%d.%d.%d"),(int)_ip_addr[0],(int)_ip_addr[1],(int)_ip_addr[2],(int)_ip_addr[3]);
				plen=fill_tcp_data(buf,plen, temp);
				//plen=fill_tcp_data_p(buf,plen,PSTR("><br>"));
				plen=fill_tcp_data_p(buf,plen,html_cbr);

				plen=fill_tcp_data_p(buf,plen,PSTR("Pwd <input name=pwd value=\""));
				plen=fill_tcp_data(buf,plen,password);
				//plen=fill_tcp_data_p(buf,plen,PSTR("\"><br>"));
				plen=fill_tcp_data_p(buf,plen,html_scbr);

				//plen=fill_tcp_data_p(buf,plen,PSTR("GW <input name=gw value="));

				eeprom_read_block ((void *)_sip_addr, (const void *)&ee_gw_addr,4);
				snprintf_P(temp,sizeof(temp),PSTR("GW <input name=gw value=%d.%d.%d.%d"),(int)_sip_addr[0],(int)_sip_addr[1],(int)_sip_addr[2],(int)_sip_addr[3]);
				plen=fill_tcp_data(buf,plen, temp);
				//plen=fill_tcp_data_p(buf,plen,PSTR("><br>"));
				//plen=fill_tcp_data_p(buf,plen,html_cbr);

				plen=fill_tcp_data_p(buf,plen,PSTR("><br>SRV <input name=sip value="));

				eeprom_read_block ((void *)_sip_addr, (const void *)&ee_sip_addr,4);
				snprintf_P(temp,sizeof(temp),PSTR("%d.%d.%d.%d:%u><br>"),(int)_sip_addr[0],(int)_sip_addr[1],(int)_sip_addr[2],(int)_sip_addr[3],_srv_port);
				plen=fill_tcp_data(buf,plen, temp);
				//plen=fill_tcp_data_p(buf,plen,PSTR("><br>"));
				//plen=fill_tcp_data_p(buf,plen,html_cbr);

				//plen=fill_tcp_data_p(buf,plen,PSTR("Script: <input name=sct maxlength=15 value=\""));
				plen=fill_tcp_data_p(buf,plen,PSTR("Script <input name=sct value=\""));
				if ( _spt[0] != (char)255 && strlen(_spt) > 0 )
				plen=fill_tcp_data(buf,plen,_spt);
				//plen=fill_tcp_data_p(buf,plen,PSTR("\"><br>"));
				plen=fill_tcp_data_p(buf,plen,html_scbr);

				/*
				plen=fill_tcp_data_p(buf,plen,PSTR("<br>Preset: <select name=pr><option value=0"));
				if (_preset != 1 )
				plen=fill_tcp_data_p(buf,plen,selected);
				// NO CLOSING TAGS!
				//plen=fill_tcp_data_p(buf,plen,PSTR(">Norm</option><option value=1"));
				plen=fill_tcp_data_p(buf,plen,PSTR(">Norm<option value=1"));
				if (_preset == 1 )
				plen=fill_tcp_data_p(buf,plen,selected);
				// NO CLOSING TAGS!
				//plen=fill_tcp_data_p(buf,plen,PSTR(">7I7O</option></select><br>"));
				plen=fill_tcp_data_p(buf,plen,PSTR(">7I7O</select><br>"));
				*/

				/*
				plen=fill_tcp_data_p(buf,plen,PSTR("Wdog: <input name=pr size=3 value="));
				if ( _preset != 255 )
				{
					itoa (_preset, temp, 10);
					plen=fill_tcp_data(buf,plen,temp);
				}
				plen=fill_tcp_data_p(buf,plen,html_cl);
				*/

				/*Temp-check*/
				//plen=fill_tcp_data_p(buf,plen,PSTR("T check: <input type=checkbox name=tc value=1"));
				//if ( eeprom_read_byte(&ee_temp_check) == 1 )
				//plen=fill_tcp_data_p(buf,plen,html_checked);
				//plen=fill_tcp_data_p(buf,plen,html_cl);
				//plen=fill_tcp_data_p(buf,plen,PSTR(" Alarm T: <input name=at size=3 value="));
				//if ( _alarm_temp != 255 && _alarm_temp > 0 )
				//{
				//	itoa (_alarm_temp, temp, 10);
				//	plen=fill_tcp_data(buf,plen,temp);
				//}
				//plen=fill_tcp_data_p(buf,plen,html_cl);

				//#1
				//sprintf(temp,"%d", readchiptemp());
				//plen=fill_tcp_data_p(buf,plen,PSTR("Cur T: "));
				//#2
				
				//sprintf(temp, "T: %d", readchiptemp());
				//plen=fill_tcp_data(buf,plen,temp);
			}
			else if ( cf_mode == 2 )
			{
				//plen=fill_tcp_data_p(buf,plen,PSTR("2>"));

				//plen=fill_tcp_data_p(buf,plen,PSTR("Megad-ID: <input name=mdid size=5 value=\""));
				plen=fill_tcp_data_p(buf,plen,PSTR("2>Megad-ID <input name=mdid value=\""));
				if ( _mdid[0] != (char)255 && strlen(_mdid) > 0 )
				plen=fill_tcp_data(buf,plen,_mdid);

				//plen=fill_tcp_data_p(buf,plen,PSTR("\"><br>"));

				plen=fill_tcp_data_p(buf,plen,PSTR("\"><br>srv loop <input type=checkbox name=sl value=1"));
				if ( eeprom_read_byte(&ee_srv_loop) == 1 )
				plen=fill_tcp_data_p(buf,plen,html_checked);
				//plen=fill_tcp_data_p(buf,plen,PSTR(" checked"));
				//plen=fill_tcp_data_p(buf,plen,PSTR(">"));
				plen=fill_tcp_data_p(buf,plen,html_cl);

			}

			//plen=fill_tcp_data_p(buf,plen,PSTR("<br><input type=submit value=Save></form>"));
			plen=fill_tcp_data_p(buf,plen,html_submit);

		}

		else if (find_key_val((char *)&(buf[dat_p+4]),gStrbuf,25,"fwup"))
		{
			eeprom_write_byte ((void*)ee_fw_up, 1);
			reset_flag = 2;
			//Bootloader();
		}

		else
		{
			//DONE:

		        plen=http200ok();

			if ( strlen(srv_cmd) > 0 )
			plen=fill_tcp_data_p(buf,plen,PSTR("Done"));
			else
			{

				//eeprom_read_block (_port_type, &ee_port_type, IO_SIZE);

				plen=fill_tcp_data_p(buf,plen,PSTR("MegaD-ArDubnino (fw: 3.57)<br><a href=/"));
				//plen=fill_tcp_data_p(buf,plen,PSTR("MegaD-328 by <a href=http://ab-log.ru>ab-log.ru</a> (fw: "));
				//plen=fill_tcp_data_p(buf,plen,PSTR(MEGADFWV));
				//plen=fill_tcp_data_p(buf,plen,PSTR(")<br><a href=/"));
				plen=fill_tcp_data(buf,plen,password);

				plen=fill_tcp_data_p(buf,plen,PSTR("/?cf=1>Config</a><br>-- Ports --"));

				//char pnum[3];
				for ( i = 0; i < IO_SIZE; i++ )
				{
					port_letter = aio[i][0];
					port_num = atoi(&aio[i][1]);

					plen=fill_tcp_data_p(buf,plen,PSTR("<br><a href=/"));
					plen=fill_tcp_data(buf,plen,password);
					plen=fill_tcp_data_p(buf,plen,html_pt);

					snprintf_P(temp,sizeof(temp),PSTR("%d"),i);
					//snprintf_P(temp,sizeof(temp),PSTR("<br><a href=/%s/?pt=%d"),password, i);

					plen=fill_tcp_data(buf,plen, temp);

					if ( port_letter == 'A' )
					{
						//plen=fill_tcp_data_p(buf,plen,PSTR(">"));
						plen=fill_tcp_data_p(buf,plen,html_cl);
						plen=fill_tcp_data(buf,plen,aio[i]);
					}
					else
					{
						plen=fill_tcp_data_p(buf,plen,PSTR(">P"));
						plen=fill_tcp_data(buf,plen,temp);
					}

					plen=fill_tcp_data_p(buf,plen,PSTR(" - "));

					if ( _port_type[i] > 3 && port_letter != 'A' )
					plen=fill_tcp_data_p(buf,plen,PSTR("NC"));
					else if ( _port_type[i] == 1 )
					//plen=fill_tcp_data_p(buf,plen,PSTR("OUT"));
					plen=fill_tcp_data_p(buf,plen,html_out);
					else if ( _port_type[i] == 0 )
					//plen=fill_tcp_data_p(buf,plen,PSTR("IN"));
					plen=fill_tcp_data_p(buf,plen,html_in);
					else if ( _port_type[i] == 2 || port_letter == 'A' )
					plen=fill_tcp_data_p(buf,plen,PSTR("ADC"));
					else if ( _port_type[i] == 3 )
					plen=fill_tcp_data_p(buf,plen,PSTR("DS"));

					plen=fill_tcp_data_p(buf,plen,PSTR("</a>"));
				}
			}

		}
		
		SENDTCP:

		LEDON;
                www_server_reply(buf,plen); // send data
		LEDOFF;

		if ( reset_flag == 1 )
		RESET();
		else if ( reset_flag == 2 )
		Bootloader();

		continue;

		UDP:
		continue;
	}

        return (0);
}
