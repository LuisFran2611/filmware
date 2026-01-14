#pragma once

#include <stdint.h>

#define ENABLE_5V	(GPOUT = (EN_5V_UP | EN_5V_M4))
#define ENABLE_1V4	(GPOUT = EN_1V4_M4)
#define DISABLE_5V_1V4	(GPOUT = 0u)

int MQ9_ReadCOppm(uint8_t channel);
int MQ9_ReadCH4ppm(uint8_t channel);

void Lectura_Gas(void);
void Lectura_C0(void);
void Lectura_CH4(void);
