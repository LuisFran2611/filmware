
#include "ModuloMQ9.h"
#include "MCP3004.h"

#define MQ9_ADC_BITS 1024
// 330 -> 3.30V scaled like the original integer conversion.
#define MQ9_ADC_SCALE 330
#define MQ9_CO_DIV 7
#define MQ9_CH4_DIV 1

static int mq9_scale_adc(int raw)
{
	return (raw * MQ9_ADC_SCALE) / MQ9_ADC_BITS;
}

static int mq9_ppm_from_scaled(int scaled, int div)
{
	if (div == 0) return 0;
	return (10 * scaled) / div;
}

int MQ9_ReadCOppm(uint8_t channel)
{
	int raw = MCP3004_Read(channel);
	int scaled = mq9_scale_adc(raw);
	return mq9_ppm_from_scaled(scaled, MQ9_CO_DIV);
}

int MQ9_ReadCH4ppm(uint8_t channel)
{
	int raw = MCP3004_Read(channel);
	int scaled = mq9_scale_adc(raw);
	return mq9_ppm_from_scaled(scaled, MQ9_CH4_DIV);
}

void Lectura_Gas(void)
{
	estado = 0;
	ENABLE_5V;
	IRQEN = IRQ_TIMER;
	MAX_COUNT = 1 * 1000000;
}

void Lectura_C0(void)
{
	int ppm;

	ppm = MQ9_ReadCOppm(MCP3004_CH0);

	DISABLE_5V_1V4;
	ENABLE_1V4;

	IRQEN = IRQ_TIMER;
	MAX_COUNT = 1 * 1000000;

	estado = 1;

	lectura_CO = ppm;
	_printf("Medicion de C0 en curso, por favor espere 60 seg.\n");
	_printf("Lectura CO: %d ppm \n", lectura_CO);
	_printf("Medicion de CH4 en curso, por favor espere 90 seg.\n");
}

void Lectura_CH4(void)
{
	int ppm;

	ppm = MQ9_ReadCH4ppm(MCP3004_CH0);

	DISABLE_5V_1V4;

	estado = 2;

	lectura_CH4 = ppm;
	_printf("Lectura CH4: %d ppm \n", lectura_CH4);
}

