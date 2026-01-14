#include "GP2Y1010.h"
#include "MCP3004.h"

int GP2Y1010_ReadRaw(uint8_t channel)
{
	int adc;

	GPOUT |= M2_ON_OFF;
	_delay_ms(GP2Y1010_SAMPLE_MS);
	adc = MCP3004_Read(channel);
	_delay_ms(GP2Y1010_LED_ON_MS - GP2Y1010_SAMPLE_MS);
	GPOUT &= ~M2_ON_OFF;

	return adc;
}
