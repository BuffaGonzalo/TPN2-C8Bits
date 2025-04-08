/*
 * hcsr04.h
 *
 * Created: 6/4/2025 16:07:48
 *  Author: gonza
 */ 
#include <stddef.h>
#include <stdint.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#define LEDBUILTIN			PB5


#ifndef HCSR04_H_
#define HCSR04_H_

void initHcSr04();

void hcSr04Task(void (*hcsr04)(), uint8_t *flags);

void triggerTask(uint8_t is100ms);

void getDistance(uint32_t *distance, uint32_t startTime, uint32_t endTime);


#endif /* HCSR04_H_ */