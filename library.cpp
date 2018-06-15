/*
 * Reloj_DS1302_0.2.cpp
 *
 * Created: 15/06/2018 02:04:33 p. m.
 * Author : jose
 */ 

#include <avr/io.h>

// INICIO DE MACROS PARA LOS PUERTOS 0-7 (PORTD, DDRD, PIND, ETC)

/* DDRD */

#define config_input_DDRD(bit)  {bit ## DDRD &= ~(1u << bit);}
#define config_output_DDRD(bit) {bit ## DDRD |= (1u << bit);}
	
/* PORTD */

#define pullup_on_PORTD(bit)    {bit ## PORTD |= (1u << bit);}
#define pullup_off_PORTD(bit)   {bit ## PORTD &= ~(1u << bit);}

/* Replace with your library code */
int myfunc(void)
{
	return 0;
}

