#pragma once

#include <stdint.h>

#define ENABLE_5V	(GPOUT = (EN_5V_UP | EN_5V_M4))
#define ENABLE_1V4	(GPOUT = EN_1V4_M4)
#define DISABLE_5V_1V4	(GPOUT = 0u)

#define MQ9_GAS_LPG 'L'
#define MQ9_GAS_CO  'C'
#define MQ9_GAS_CH4 'H'

#define MQ9_OUT_CO  0
#define MQ9_OUT_LPG 1
#define MQ9_OUT_CH4 2

void MQ9_Init(void);
void MQ9_SetADC(uint16_t adc_bits, float adc_voltage);
void MQ9_SetR0(float r0);
float MQ9_GetR0(void);
float MQ9_Calibrate(uint8_t channel);
float MQ9_GetValue(uint8_t channel, char gasName, int printing, float *output);
int MQ9_ThresholdValue(uint8_t channel, char gasName, float threshold);
int MQ9_ThresholdRaw(uint8_t channel, int threshold);

void Lectura_Gas(void);
void Lectura_C0(void);
void Lectura_CH4(void);
