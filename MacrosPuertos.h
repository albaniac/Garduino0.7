/*
 * Control_Puertos2.c
 *
 * Created: 16/06/2018 06:15:04 p. m.
 * Author : jose
 
 referencias:
 https://www.embedded.com/design/programming-languages-and-tools/4397803/Interrupts-short-and-simple--Part-1---Good-programming-practices
 https://www.geeksforgeeks.org/enumeration-enum-c/
 https://www.nongnu.org/avr-libc/user-manual/group__avr__interrupts.html
 
 macros sacados de:
 https://www.avrfreaks.net/forum/macros-ddr-and-port-pin-name
 https://www.avrfreaks.net/forum/tut-c-bit-manipulation-aka-programming-101?page=all
 https://www.arduino.cc/reference/en/language/functions/digital-io/pinmode/
 
 http://graphics.stanford.edu/~seander/bithacks.html
 https://www.nongnu.org/avr-libc/user-manual/group__avr__interrupts.html#gaad5ebd34cb344c26ac87594f79b06b73
 https://www.arduino.cc/reference/en/language/functions/digital-io/digitalread/ 
 
 https://gmplib.org/manual/Low_002dlevel-Functions.html#Low_002dlevel-Functions
 https://www.geeksforgeeks.org/interesting-facts-bitwise-operators-c/
 https://www.geeksforgeeks.org/bitwise-hacks-for-competitive-programming/
 
 
 https://www.gnu.org/software/gawk/manual/html_node/Bitwise-Functions.html
 
 */ 

#ifndef MacrosPuertos_H
#define MacrosPuertos_H

#define HIGH 0x1
#define LOW  0x0

#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define EULER 2.718281828459045235360287471352

#define SERIAL  0x0
#define DISPLAY 0x1

//--------------- definiciones para utilizar banderas en las manipulaciones de puertos --
#define NOT_A_PIN 0
#define NOT_A_PORT 0
#define NOT_AN_INTERRUPT -1

//---------------------------------------------------------------------------------

#define digitalPinToPort(P) ( pgm_read_byte( digital_pin_to_port_PGM + (P) ) )
#define digitalPinToBitMask(P) ( pgm_read_byte( digital_pin_to_bit_mask_PGM + (P) ) )
#define digitalPinToTimer(P) ( pgm_read_byte( digital_pin_to_timer_PGM + (P) ) )
#define analogInPinToBit(P) (P)
#define portOutputRegister(P) ( (volatile uint8_t *)( pgm_read_word( port_to_output_PGM + (P))) )
#define portInputRegister(P) ( (volatile uint8_t *)( pgm_read_word( port_to_input_PGM + (P))) )
#define portModeRegister(P) ( (volatile uint8_t *)( pgm_read_word( port_to_mode_PGM + (P))) )

//------------- define minimos y maximos----------------//
#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))

//-------------calculos matematicos rapidos --------------//

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define round(x)     ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)
#define sq(x) ((x)*(x))

//------------test de byte----------//

#define lowByte(w) ((uint8_t) ((w) & 0xff))
#define highByte(w) ((uint8_t) ((w) >> 8UL))

//-------------------funciones BitRead ----------------//

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1U << (bit)))
#define bitClear(value, bit) ((value) &= ~(1U << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

//--------macros para bit mask de puertos 

/*
	DDRx - The Port x Data Direction Register - read/write
	pinMode(3 (DAT), OUTPUT);	 
		0 = INPUT - write
		1 = OUTPUT - read
*/
#define CONFIG_AS_INPUT(DDR)                {DDR &= ~(1U << DDR);}
#define CONFIG_AS_OUTPUT(DDR)               {DDR |= (1U << DDR);}	

/*
	PORTx - The Port x Data Register - read/write
	digitalWrite(3 (DAT), LOW); 
		0 = LOW - write
		1 = HIGH - read
*/

#define PULLUP_ON(PORT)                     {PORT |= (1U << PORT);}
#define PULLUP_OFF(PORT)                    {PORT &= ~(1U << PORT);}

#define disable_digital_input(channel)      {DIDR0 |= (1U << channel);}
#define enable_digital_input(channel)       {DIDR0 &= ~(1U << channel);}

#define enable_pin_change_interrupt(PCMSK)  {PCMSK |= (1U << PCINT);}
#define disable_pin_change_interrupt(PCMSK) {PCMSK &= ~(1U << PCINT);}
	
/***** Manipulate Outputs *****/
#define set_high(PORT)                      {PORT |= (1U << PORT);}
#define set_low(PORT)                       {PORT &= ~(1U << PORT);}

/*
	PINx - The Port x Input Pins Register - read only
	
		Dicta la direccion binaria del input/output de ese pueto
	
*/
	
#define toggle(PIN)                         {PIN |= (1U << PIN);}
	
/***** Test Inputs *****/
#define PIN_is_high(PIN)                    (PIN &= (1U << PIN))
#define PIN_is_low(PIN)                     (! (PIN &= (1U << PIN)))

//contacto "de lejos" para PORTD (0 - 7)
#define D_REED_CONTACT_DDR					DDRD
#define D_REED_CONTACT_PORT					PORTD
#define D_REED_CONTACT_PIN					PIND

//contacto "de lejos" para PORTB (8 - 13)
#define B_REED_CONTACT_DDR					DDRB
#define B_REED_CONTACT_PORT					PORTB
#define B_REED_CONTACT_PIN					PINB

//contacto "de lejos" para PORTC (A0 - A5)
#define C_REED_CONTACT_DDR					DDRC
#define C_REED_CONTACT_PORT					PORTC
#define C_REED_CONTACT_PIN					PINC

#define REED_CONTACT_PCMSK					PCMSK0
#define REED_CONTACT_PCINT					PCINT0

#endif

/*
//guia rapida
PORTx{
	DDRx - The Port x Data Direction Register - read/write{
		0 = INPUT - write
		1 = OUTPUT - read
	}
	
	PORTx - The Port x Data Register - read/write{
		0 = LOW - write
		1 = HIGH - read
	}
	
	PINx - The Port x Input Pins Register - read only{
		B76543210 = B00000000
		Dicta la direccion binaria del input/output de ese pueto
	}
}

ejemplo:
DDRD = B11111110;  // sets Arduino pins 1 to 7 as outputs, pin 0 as input
DDRD = DDRD | B11111100;  // this is safer as it sets pins 2 to 7 as outputs
// without changing the value of pins 0 & 1, which are RX & TX

CLK = set clock (out) / input del reloj
CE = chip enable (out) / input del reloj
DAT = data transfer (input)/output

CLK -> 4
DAT <-> 3 //bitmap para B00000000
CE -> 2

ejemplo:
-----------------------------------------			   76543210
digitalWrite(3 (DAT), LOW);  PORTD = PORTD | PORTD = B00000000;
pinMode(3 (DAT), OUTPUT);	    DDRD = DDRD | DDRD  = B00110000;
--------------------------------------------
*/