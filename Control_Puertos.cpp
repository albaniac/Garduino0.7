/*
 * Control_Puertos_3.cpp
 *
 * Created: 16/06/2018 09:10:46 p. m.
 * Author : jose
 */ 

#include <avr/io.h>
#include "Control_Puertos.h"
#include "MacrosPuertos.h" //esta libreria contiene los macros necesarios para port mapping


enum CheckMaskFlag{
	// Enumeration (or enum) is a user defined data type in C.
	// It is mainly used to assign names to integral constants,
	// the names make a program easy to read and maintain.

	/*4)*/ RELOJ_CLK = 0b00010000,
	/*3)*/ RELOJ_DAT = 0b00001000,
	/*2)*/ RELOJ_RST = 0b00000100
	
};

void bit_map::Config_as_input_DDRD(uint8_t DDR){

	if(DDR == NOT_A_PIN) return;
	//revisa si bit == 0
	
	//volatile uint8_t *registro;
	
	if (DDR & (RELOJ_CLK | RELOJ_DAT | RELOJ_RST)){
		//revisa si el bit se encuentra en una de las direcciones del reloj
		cli();
		CONFIG_AS_INPUT(D_REED_CONTACT_DDR);
	}
}

void bit_map::Config_as_output_DDRD(uint8_t DDR){
	
	if(DDR == NOT_A_PIN) return;
	//revisa si bit == 0
		
	//volatile uint8_t *registro;
		
	if (DDR & (RELOJ_CLK | RELOJ_DAT | RELOJ_RST)){
		//revisa si el bit se encuentra en una de las direcciones del reloj
		cli();
		CONFIG_AS_OUTPUT(D_REED_CONTACT_DDR);
	}
}

void bit_map::Pullup_on_DDRD(uint8_t PORT){
	
	if(PORT == NOT_A_PIN) return;
	//revisa si bit == 0
	
	//volatile uint8_t *registro;
	
	if (PORT & (RELOJ_CLK | RELOJ_DAT | RELOJ_RST)){
		//revisa si el bit se encuentra en una de las direcciones del reloj
		cli();
		PULLUP_ON(D_REED_CONTACT_PORT);
	}	
}

void bit_map::Pullup_off_DDRD(uint8_t PORT){
	
	if(PORT == NOT_A_PIN) return;
	//revisa si bit == 0
	
	//volatile uint8_t *registro;
	
	if (PORT & (RELOJ_CLK | RELOJ_DAT | RELOJ_RST)){
		//revisa si el bit se encuentra en una de las direcciones del reloj
		cli();
		PULLUP_OFF(D_REED_CONTACT_PORT);
	}
}

void bit_map::TogglePIND(uint8_t PIN){
	
	if(PIN == NOT_A_PIN) return;
	//revisa si bit == 0
		
	//volatile uint8_t *registro;
		
	if (PIN & (RELOJ_CLK | RELOJ_DAT | RELOJ_RST)){
		//revisa si el bit se encuentra en una de las direcciones del reloj
		cli();
		toggle(D_REED_CONTACT_PIN);
	}
}

void bit_map::ReadLow(uint8_t PIN){
	
	if(PIN == NOT_A_PIN) return;
	//revisa si bit == 0
		
	//volatile uint8_t *registro;
		
	if (PIN & (RELOJ_CLK | RELOJ_DAT | RELOJ_RST)){
		//revisa si el bit se encuentra en una de las direcciones del reloj
		cli();
		PIN_is_low(D_REED_CONTACT_PIN);
	}

}

void bit_map::ReadHigh(uint8_t PIN){
	
	if(PIN == NOT_A_PIN) return;
	//revisa si bit == 0
		
	//volatile uint8_t *registro;
		
	if (PIN & (RELOJ_CLK | RELOJ_DAT | RELOJ_RST)){
		//revisa si el bit se encuentra en una de las direcciones del reloj
		cli();
		PIN_is_low(D_REED_CONTACT_PIN);
	}
}