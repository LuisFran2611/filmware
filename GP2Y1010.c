#include "GP2Y1010.h"
#include "MCP3004.h"

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

	GPOUT |= M2_ON_OFF;
	while ((uint32_t)(TIMER - start) < sample_ticks) {
	}
	adc = MCP3004_Read(channel);
	while ((uint32_t)(TIMER - start) < total_ticks) {
	}
	GPOUT &= ~M2_ON_OFF;

	return adc;
}
