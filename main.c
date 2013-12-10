/*
 * main.c
 *
 *  Created on: 17 paü 2013
 *      Author: Karol
 */
#include<avr/io.h>
#include<util/delay.h>
#include<avr/interrupt.h>
#include<stdio.h>
#include<stddef.h>
#include<nRF24L01.h>
#include<hd44780.h>
//Definicje
#define SET_FORWARD PORTA |= _BV(0)
#define RESET_FORWARD PORTA &= ~_BV(0)
#define SET_REVERSE PORTA |= _BV(1)
#define RESET_REVERSE PORTA &=~ _BV(1)
#define RESET_DIR PORTA &= ~((_BV(0))|(_BV(1)))
#define FR_SPEED OCR1A
#define SET_RIGHT PORTA |= _BV(2)
#define RESET_RIGHT PORTA &= ~_BV(2)
#define SET_LEFT PORTA |= _BV(3)
#define RESET_LEFT PORTA &=~ _BV(3)
#define RESET_TURN PORTA &=~((_BV(2))|(_BV(3)))
#define RL_SPEED OCR1B
#define LED_ON PORTA &= ~_BV(4)
#define LED_OFF PORTA |= _BV(4)

#define SET_CSN PORTB |= _BV(3)
#define CLEAR_CSN PORTB &= ~_BV(3)
#define SET_CE PORTB |= _BV(2)
#define CLEAR_CE PORTB &= ~_BV(2)
#define W 1
#define R 0

//Zmienne
char *receive_buffer;
uint8_t X_acc;
uint8_t Y_acc;
uint8_t Z_acc;
char Text_buff[5];

//W≥asne funkcje

void init_ster(void)
{

	DDRA |= (1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3) | (1 << PB4);
	DDRD |= (1 << PD4) | (1 << PD5);
	TCCR1A |= (1 << COM1A1) | (1<< COM1B1) | (1 << WGM10);
	TCCR1B |= (1 << WGM12) | (1 << CS10) | (1 << CS11);
	OCR1A = 0;
	OCR1B = 0;


}

void ster(uint8_t X_acceleration , uint8_t Y_acceleration)
{
	if((X_acceleration > 128) & (X_acceleration <= 255))
	{
	RESET_DIR;
	SET_FORWARD;
	OCR1B = 3*(255-X_acceleration);
	}
	if((X_acceleration >= 0) & (X_acceleration <= 128))
	{
	RESET_DIR;
	SET_REVERSE;
	OCR1B = 3*X_acceleration;
	}
	if((Y_acceleration > 128) & (Y_acceleration <= 255))
	{
	RESET_TURN;
	SET_LEFT;
	if((255-Y_acceleration)>20) OCR1A = 230;
	else OCR1A = 0;
	}
	if((Y_acceleration >= 0) & (Y_acceleration <= 128))
	{
	RESET_TURN;
	SET_RIGHT;
	if(Y_acceleration > 20) OCR1A = 230;
	else OCR1A = 0;
	}
}
/*
void init_USART(void)
{
	#define BAUD 9600// set baudrate
    #include<util/setbaud.h>
	UBRRH = UBRRH_VALUE;
	UBRRL = UBRRL_VALUE;
	#if USE_2x
	UCSRA |= (1 << U2X);
	#else
	UCSRA &= ~(1 << U2X);
	#endif
	UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);//8 bits of data, 1 stop bit, parity none
	UCSRB = (1 << TXEN) | (1 << RXEN) ;


}

ISR(USART_RXC_vect)
{
	//przerwanie po nadejsciu bajtu danych
}

ISR(USART_UDRE_vect)
{
	//przerwansie generowane gdy bufor nadawania jest juz pusty
}
*/
/*
void send_stringUSART(uint8_t* data, unsigned char length)
{
	unsigned char i;
	for(i = 0;i < length;i++)
	{
		send_USART(data[i]);
	}
}
void send_USART(uint8_t data)
{

	while(!(UCSRA & (1 << UDRE)));//wait untill sending bufor is empty
	UDR = data;
}
*/

void init_SPI(void)
{
	/*
	 * SCK(PB7) as output
	 * MISO(PB6) as input
	 * MOSI(PB5) as output
	 * CSN(PB3) as output
	 * CE(PB2) as output
	 *
	 */

	DDRB |= (1<<PB7)|(1<<PB5)|(1<<PB3)|(1<<PB2)|(1<<PB4);
	SPCR |= (1<<SPE)|(1<<MSTR)|(1<<SPR1)|(1<<SPR0);
	SET_CSN;
	CLEAR_CE;
}


uint8_t WriteByteSPI(uint8_t Data)
{
	SPDR = Data;//load date for send
	while(!(SPSR & (1<<SPIF)));//wait sending in progress
	return SPDR;//return data that was resend
}


uint8_t GetRegister(uint8_t reg)
{
	_delay_us(10);
	CLEAR_CSN;
	_delay_us(10);
	WriteByteSPI(R_REGISTER + reg);
	_delay_us(10);
	reg=WriteByteSPI(NOP);
	_delay_us(10);
	SET_CSN;
	return reg;
}

void WriteToNrf(uint8_t reg, uint8_t data)
{
	_delay_us(10);
	CLEAR_CSN;
	_delay_us(10);
	WriteByteSPI(W_REGISTER + reg);
	_delay_us(10);
	WriteByteSPI(data);
	_delay_us(10);
	SET_CSN;
}

uint8_t* WriteReadToNrf(uint8_t ReadWrite, uint8_t reg, uint8_t *DataRW, uint8_t DataRWlength)
{
	if(ReadWrite == W)
	{
		reg = W_REGISTER + reg;
	}
	static uint8_t ret[32];

	_delay_us(10);
	CLEAR_CSN;
	_delay_us(10);
	WriteByteSPI(reg);
	_delay_us(10);

	int i;
	for(i = 0; i < DataRWlength; i++)
	{
		if(ReadWrite ==R && reg != W_TX_PAYLOAD)
		{
			ret[i] = WriteByteSPI(NOP);
			_delay_us(10);
		}
		else
		{
			WriteByteSPI(DataRW[i]);
			_delay_us(10);
		}
	}
	SET_CSN;
	return ret;
}

void NRF24L01_init(void)
{
	_delay_ms(100);//Opoznienie 100ms
	uint8_t val[5];

	val[0]=0x01;
	WriteReadToNrf(W,EN_AA,val,1);// Wlaczenie auto powiadomiania o dotarci danych

	val[0]=0x2F;
	WriteReadToNrf(W,SETUP_RETR,val,1);//15 powtorzen w razie niepowodzenia odbioru, co 750 us

	val[0]=0x01;
	WriteReadToNrf(W,EN_RXADDR,val,1);//linia danych 0

	val[0]=0x03;
	WriteReadToNrf(W,SETUP_AW,val,1);//adres ma miec 5 bajtow

	val[0]=0x01;
	WriteReadToNrf(W,RF_CH,val,1);//czestotliwosc 2.401GHz

	val[0]=0x27;
	WriteReadToNrf(W,RF_SETUP,val,1);// predkosc 1 Mbs

	int i;
	for(i = 0; i < 5; i++)
	{
		val[i]=0x12;
	}
	WriteReadToNrf(W,RX_ADDR_P0,val,5);//ustalenie adresu na lini numer 0
	for(i = 0; i < 5; i++)
	{
		val[i]=0x12;
	}
	WriteReadToNrf(W,TX_ADDR,val,5);// ustalenie adresu nadajnika

	val[0]=5;
	WriteReadToNrf(W,RX_PW_P0,val,1);// paczka danych ma miec 5 bajtow

	val[0]=0x1F;
	WriteReadToNrf(W,CONFIG,val,1);//Uruchomienie w trybie odbiornika

	_delay_ms(100);

}

void transmit_payload(uint8_t *Send_buffor )
{
	WriteReadToNrf(R,FLUSH_TX,Send_buffor,0);
	WriteReadToNrf(R,W_TX_PAYLOAD,Send_buffor,5);
	_delay_ms(10);
	SET_CE;
	_delay_us(100);
	CLEAR_CE;
	_delay_ms(10);
}

void receive_payload(void)
{
	SET_CE;
	_delay_ms(100);
	CLEAR_CE;
}

void reset(void)
{
	_delay_us(10);
	CLEAR_CSN;
	_delay_us(10);
	WriteByteSPI(W_REGISTER + STATUS);
	_delay_us(10);
	WriteByteSPI(0x70);
	_delay_us(10);
	SET_CSN;
}


int main(void)
{
	//init_USART();
	init_SPI();
	init_ster();
	lcd_init();
	LCD_DISPLAY(LCDDISPLAY);
	lcd_puts("KAROL");

	if(GetRegister(STATUS) == 0x0E)
	{
		LED_ON;
		_delay_ms(1000);
		LED_OFF;
		_delay_ms(1000);
	}
	LCD_CLEAR;
	NRF24L01_init();
	while(1)
	{
		reset();
		receive_payload();
		if(((GetRegister(STATUS) & (1<<RX_DR)) != 0))
		{
		LED_ON;
		_delay_ms(10);
		//LCD_CLEAR;
		receive_buffer=WriteReadToNrf(R,R_RX_PAYLOAD,receive_buffer,5);
		X_acc=receive_buffer[0];
		Y_acc=receive_buffer[1];
		Z_acc=receive_buffer[2];
		ster(X_acc,Y_acc);
		LCD_LOCATE(0,0);
		sprintf(Text_buff,"%3d",X_acc);
		lcd_puts(Text_buff);
		LCD_LOCATE(0,1);
		sprintf(Text_buff,"%3d",Y_acc);
		lcd_puts(Text_buff);
		LED_OFF;
		}

	}
}


