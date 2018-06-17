#ifndef Control_Puertos_H
#define Control_Puertos_H

#include <avr/pgmspace.h>
#include <avr/interrupt.h>

class bit_map{
	public:
//----------------------------PORT D--------------------------------
		//DDRx - The Port x Data Direction Register - read/write
		//convierte los puertos de input a output
		void Config_as_input_DDRD(uint8_t DDR);
		void Config_as_output_DDRD(uint8_t DDR); //pinMode(3 (DAT), OUTPUT);
		
		//PORTx - The Port x Data Register - read/write
		//activa los resistores internos del arduino
		//convierte la señal de 5 a 2V
		void Pullup_on_DDRD(uint8_t PORT);
		void Pullup_off_DDRD(uint8_t PORT); //digitalWrite(3 (DAT), LOW); 
		
		//puedes utilizarlo como pinmode para ajustarlo como low o high
		//TogglePIND(Config_as_output_DDRD(DS1302_IO_PIN)_); 
		void TogglePIND(uint8_t PIN); //pinmode
		void ReadHigh(uint8_t PIN);	//digitalRead(PIN_HIGH)
		void ReadLow(uint8_t PIN); //digitalRead(PIN_LOW)
};

#endif