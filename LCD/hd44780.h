/*
Plik hd44780.h
*/

#ifndef LCD_HD44780
#define LCD_HD44780

/* RS */
#define SET_OUT_LCD_RS  DDRC  |=  _BV(PC0)
#define SET_LCD_RS      PORTC |=  _BV(PC0)
#define CLR_LCD_RS      PORTC &= ~_BV(PC0)

/* RW */
#define SET_OUT_LCD_RW  DDRC  |=  _BV(PC1)
#define SET_LCD_RW      PORTC |=  _BV(PC1)
#define CLR_LCD_RW      PORTC &= ~_BV(PC1)

/* E */
#define SET_OUT_LCD_E   DDRC  |=  _BV(PC2)
#define SET_LCD_E       PORTC |=  _BV(PC2)
#define CLR_LCD_E       PORTC &= ~_BV(PC2)

/* D4 */
#define SET_OUT_LCD_D4  DDRC  |=  _BV(PC3)
#define SET_IN_LCD_D4   DDRC  &= ~_BV(PC3)
#define SET_LCD_D4      PORTC |=  _BV(PC3)
#define CLR_LCD_D4      PORTC &= ~_BV(PC3)
#define IS_SET_LCD_D4   PINC  &   _BV(PC3)

/* D5 */
#define SET_OUT_LCD_D5  DDRC  |=  _BV(PC4)
#define SET_IN_LCD_D5   DDRC  &= ~_BV(PC4)
#define SET_LCD_D5      PORTC |=  _BV(PC4)
#define CLR_LCD_D5      PORTC &= ~_BV(PC4)
#define IS_SET_LCD_D5   PINC  &   _BV(PC4)

/* D6 */
#define SET_OUT_LCD_D6  DDRC  |=  _BV(PC5)
#define SET_IN_LCD_D6   DDRC  &= ~_BV(PC5)
#define SET_LCD_D6      PORTC |=  _BV(PC5)
#define CLR_LCD_D6      PORTC &= ~_BV(PC5)
#define IS_SET_LCD_D6   PINC  &   _BV(PC5)

/* D7 */
#define SET_OUT_LCD_D7  DDRC  |=  _BV(PC6)
#define SET_IN_LCD_D7   DDRC  &= ~_BV(PC6)
#define SET_LCD_D7      PORTC |=  _BV(PC6)
#define CLR_LCD_D7      PORTC &= ~_BV(PC6)
#define IS_SET_LCD_D7   PINC  &   _BV(PC6)


#define LCD_NOP asm volatile("nop\n\t""nop\n\t" "nop\n\t" "nop\n\t" ::);



#define LCDCOMMAND 0
#define LCDDATA    1

#define LCD_LOCATE(x,y)  WriteToLCD(0x80|((x)+((y)*0x40)), LCDCOMMAND)

#define LCD_CLEAR              WriteToLCD(0x01, LCDCOMMAND)
#define LCD_HOME               WriteToLCD(0x02, LCDCOMMAND)

/* IDS */

#define LCDINCREMENT           0x02
#define LCDDECREMENT           0x00
#define LCDDISPLAYSHIFT        0x01

#define LCD_ENTRY_MODE(IDS)    WriteToLCD(0x04|(IDS), LCDCOMMAND)

/* BCD */
#define LCDDISPLAY             0x04
#define LCDCURSOR              0x02
#define LCDBLINK               0x01

#define LCD_DISPLAY(DCB)       WriteToLCD(0x08|(DCB), LCDCOMMAND)

/* RL */
#define LCDLEFT                0x00
#define LCDRIGHT               0x04

#define LCD_SHIFT_DISPLAY(RL)  WriteToLCD(0x18|(RL), LCDCOMMAND)
#define LCD_SHIFT_CURSOR(RL)   WriteToLCD(0x10|(RL), LCDCOMMAND)

#define LCD_CGRAM_ADDRESS(A)   WriteToLCD(0x40|((A)&0x3f), LCDCOMMAND)
#define LCD_DDRAM_ADDRESS(A)   WriteToLCD(0x80|((A)&0x7f), LCDCOMMAND)

#define LCD_WRITE_DATA(D)      WriteToLCD((D),LCDDATA)


void lcd_init(void);
void WriteToLCD(unsigned char v,unsigned char rs);
unsigned char ReadAddressLCD(void);
void lcd_puts(char *str);

#endif
