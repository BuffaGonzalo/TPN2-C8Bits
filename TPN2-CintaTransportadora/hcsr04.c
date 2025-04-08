/*
 * hcsr04.c
 *
 * Created: 6/4/2025 16:02:12
 *  Author: gonza
 */ 

#include "hcsr04.h"

typedef union{
	struct{
		uint8_t bit7 : 1;
		uint8_t bit6 : 1;
		uint8_t bit5 : 1;
		uint8_t bit4 : 1;
		uint8_t bit3 : 1;
		uint8_t bit2 : 1;
		uint8_t bit1 : 1;
		uint8_t bit0 : 1;
	}bits;
	uint8_t bytes;
}_uFlag;

//define
#define RESETFLAGS          hcFlags.bytes
#define OKDISTANCE			hcFlags.bits.bit0
#define TRIGGERDONE			hcFlags.bits.bit1

#define NEWMEASURE			hcFlags.bits.bit3

#define IS100MS				hcFlags.bits.bit7

//Definición de variables
_uFlag hcFlags;
uint8_t ISDISTANCE = 0;
uint8_t CLRFLAGS = 0;

void initHcSr04()
{
	//inicialización de las banderas
	RESETFLAGS = 0;
}

void hcSr04Task(void (*hcsr04)(), uint8_t *flags) //
{
	hcFlags.bytes = *flags; //tomamos los valores 
	//ejecución continua de la función
	
	triggerTask(IS100MS);
	
	if(CLRFLAGS)
	{
		OKDISTANCE = 0; //hacemos 0 aca para un control correcto
		TRIGGERDONE = 0;
		CLRFLAGS = 0;
	}
	
	//Sistema de control
	if(OKDISTANCE && TRIGGERDONE)
	{
		ISDISTANCE=1; 
	}	
	
	*flags = hcFlags.bytes;	//guardamos los valores 
	hcsr04(); //ejecución de la función
}

void triggerTask(uint8_t is100ms)
{
	if(is100ms && ~TRIGGERDONE)//ejecucion cada 100ms y que no haya ocurrido un trigger
		NEWMEASURE=1;
}

uint32_t getDistance(uint32_t startTime, uint32_t endTime)
{
	uint32_t distance = 0;

	if(ISDISTANCE)
	{
		//Utilizamos estas 2 banderas para poder setear en distintos momentos 
		ISDISTANCE = 0;
		CLRFLAGS = 1;
		distance = (endTime - startTime)/2; //distancia [cm]
		return distance;
	}
	return 0;	
}

