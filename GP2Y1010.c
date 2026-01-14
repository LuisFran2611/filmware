#include "GP2Y1010.h"
#include "MCP3004.h"

extern int _printf(const char *format, ...);

static uint32_t gp2y1010_ms_to_ticks(uint32_t ms)
{
	return (CCLK / 1000u) * ms;
}

int GP2Y1010_ReadRaw(uint8_t channel)
{
	uint32_t sample_ticks = gp2y1010_ms_to_ticks(GP2Y1010_SAMPLE_MS);
	uint32_t total_ticks = gp2y1010_ms_to_ticks(GP2Y1010_LED_ON_MS);
	uint32_t start = TIMER;
	int adc;

	// DEBUG: encendiendo LED.
	GPOUT |= M2_ON_OFF;
	_printf("GP2Y1010: LED ON\n");
	// DEBUG: esperando a que el contador llegue al punto de muestreo.
	while ((uint32_t)(TIMER - start) < sample_ticks) {
	}
	// DEBUG: contador pasado, tomar ADC.
	adc = MCP3004_Read(channel);
	_printf("GP2Y1010: sample=%d\n", adc);
	// DEBUG: manteniendo LED encendido hasta completar la ventana.
	while ((uint32_t)(TIMER - start) < total_ticks) {
	}
	// DEBUG: apagando LED.
	GPOUT &= ~M2_ON_OFF;
	_printf("GP2Y1010: LED OFF\n");

	return adc;
}
