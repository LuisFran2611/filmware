
#include "ModuloMQ9.h"
#include "MCP3004.h"

#define MQ9_LN2   0.69314718056f
#define MQ9_LN10  2.30258509299f
#define MQ9_INV_LN2 1.44269504089f
#define MQ9_EXP_MAX 88.0f

static float mq9_r0 = 10.0f;
static uint16_t mq9_adc_bits = 1024;
static float mq9_adc_voltage = 3.3f;

// Lightweight log/exp approximations to avoid libm.
static float mq9_frexp(float x, int *exp)
{
	union {
		float f;
		uint32_t u;
	} v;
	uint32_t e;

	if (x <= 0.0f) {
		*exp = 0;
		return 0.0f;
	}

	v.f = x;
	e = (v.u >> 23) & 0xFF;
	if (e == 0) {
		*exp = 0;
		return 0.0f;
	}

	*exp = (int)e - 127;
	v.u = (v.u & 0x807FFFFF) | 0x3F000000;
	return v.f;
}

static float mq9_ldexp(float x, int exp)
{
	union {
		float f;
		uint32_t u;
	} v;
	int e;

	if (x == 0.0f) return 0.0f;
	v.f = x;

	e = (int)((v.u >> 23) & 0xFF);
	if (e == 0) return 0.0f;

	e += exp;
	if (e <= 0) return 0.0f;
	if (e >= 255) return (x > 0.0f) ? 3.4e38f : -3.4e38f;

	v.u = (v.u & 0x807FFFFF) | ((uint32_t)e << 23);
	return v.f;
}

static float mq9_logf(float x)
{
	int exp;
	float mant;
	float y;
	float y2;
	float ln_mant;

	mant = mq9_frexp(x, &exp);
	if (mant <= 0.0f) return -3.4e38f;

	y = (mant - 1.0f) / (mant + 1.0f);
	y2 = y * y;
	ln_mant = 2.0f * (y + (y2 * y) / 3.0f + (y2 * y2 * y) / 5.0f);

	return ln_mant + ((float)exp * MQ9_LN2);
}

static float mq9_expf(float x)
{
	int n;
	float r;
	float r2;
	float exp_r;

	if (x > MQ9_EXP_MAX) return 3.4e38f;
	if (x < -MQ9_EXP_MAX) return 0.0f;

	n = (int)(x * MQ9_INV_LN2 + (x >= 0.0f ? 0.5f : -0.5f));
	r = x - ((float)n * MQ9_LN2);
	r2 = r * r;
	exp_r = 1.0f + r + (r2 * 0.5f) + (r2 * r / 6.0f) + (r2 * r2 / 24.0f);

	return mq9_ldexp(exp_r, n);
}

static float mq9_pow10f(float x)
{
	return mq9_expf(x * MQ9_LN10);
}

static float mq9_ratio_from_adc(int adc_value)
{
	float sensor_volt;
	float rs_gas;

	if (adc_value <= 0) return 0.0f;

	sensor_volt = ((float)adc_value / (float)mq9_adc_bits) * mq9_adc_voltage;
	if (sensor_volt <= 0.0f) return 0.0f;

	rs_gas = (mq9_adc_voltage - sensor_volt) / sensor_volt;
	return rs_gas / mq9_r0;
}

static float mq9_ppm_from_ratio(float ratio, float m, float b)
{
	if (ratio <= 0.0f) return 0.0f;
	return mq9_pow10f((mq9_logf(ratio) - b) / m);
}

static void mq9_print_value(const char *label, float value)
{
	int scaled;
	int whole;
	int frac;

	scaled = (int)(value * 100.0f + (value >= 0.0f ? 0.5f : -0.5f));
	whole = scaled / 100;
	frac = scaled % 100;
	if (frac < 0) frac = -frac;
	_printf("%s%d.%02d\n", label, whole, frac);
}

void MQ9_Init(void)
{
	mq9_adc_bits = 1024;
	mq9_adc_voltage = 3.3f;
}

void MQ9_SetADC(uint16_t adc_bits, float adc_voltage)
{
	if (adc_bits == 0 || adc_voltage <= 0.0f) return;
	mq9_adc_bits = adc_bits;
	mq9_adc_voltage = adc_voltage;
}

void MQ9_SetR0(float r0)
{
	if (r0 > 0.0f) mq9_r0 = r0;
}

float MQ9_GetR0(void)
{
	return mq9_r0;
}

float MQ9_Calibrate(uint8_t channel)
{
	uint16_t i;
	uint32_t sum = 0;
	float sensor_value;
	float sensor_volt;
	float rs_air;

	for (i = 0; i < 1000; i++) {
		sum += (uint32_t)MCP3004_Read(channel);
	}

	sensor_value = (float)sum / 1000.0f;
	if (sensor_value <= 0.0f) return mq9_r0;

	sensor_volt = (sensor_value / (float)mq9_adc_bits) * mq9_adc_voltage;
	rs_air = (mq9_adc_voltage - sensor_volt) / sensor_volt;
	mq9_r0 = rs_air / 9.9f;

	return mq9_r0;
}

float MQ9_GetValue(uint8_t channel, char gasName, int printing, float *output)
{
	int adc_value;
	float ratio;
	float ppm_co, ppm_lpg, ppm_ch4;

	adc_value = MCP3004_Read(channel);
	ratio = mq9_ratio_from_adc(adc_value);
	if (ratio <= 0.0f) {
		if (output) {
			output[MQ9_OUT_CO] = 0.0f;
			output[MQ9_OUT_LPG] = 0.0f;
			output[MQ9_OUT_CH4] = 0.0f;
		}
		return 0.0f;
	}

	switch (gasName) {
	case MQ9_GAS_CO:
		ppm_co = mq9_ppm_from_ratio(ratio, -0.50386f, 1.41558f);
		if (output) output[MQ9_OUT_CO] = ppm_co;
		if (printing) mq9_print_value("CO: ", ppm_co);
		return ppm_co;
	case MQ9_GAS_LPG:
		ppm_lpg = mq9_ppm_from_ratio(ratio, -0.5157f, 1.56831f);
		if (output) output[MQ9_OUT_LPG] = ppm_lpg;
		if (printing) mq9_print_value("LPG: ", ppm_lpg);
		return ppm_lpg;
	case MQ9_GAS_CH4:
		ppm_ch4 = mq9_ppm_from_ratio(ratio, -0.372003f, 1.33309f);
		if (output) output[MQ9_OUT_CH4] = ppm_ch4;
		if (printing) mq9_print_value("CH4: ", ppm_ch4);
		return ppm_ch4;
	default:
		ppm_co = mq9_ppm_from_ratio(ratio, -0.50386f, 1.41558f);
		ppm_lpg = mq9_ppm_from_ratio(ratio, -0.5157f, 1.56831f);
		ppm_ch4 = mq9_ppm_from_ratio(ratio, -0.372003f, 1.33309f);
		if (output) {
			output[MQ9_OUT_CO] = ppm_co;
			output[MQ9_OUT_LPG] = ppm_lpg;
			output[MQ9_OUT_CH4] = ppm_ch4;
		}
		if (printing) {
			mq9_print_value("CO: ", ppm_co);
			mq9_print_value("LPG: ", ppm_lpg);
			mq9_print_value("CH4: ", ppm_ch4);
		}
		return 0.0f;
	}
}

int MQ9_ThresholdValue(uint8_t channel, char gasName, float threshold)
{
	float ppm = MQ9_GetValue(channel, gasName, 0, 0);
	return ppm >= threshold;
}

int MQ9_ThresholdRaw(uint8_t channel, int threshold)
{
	return MCP3004_Read(channel) >= threshold;
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
	float ppm;

	ppm = MQ9_GetValue(MCP3004_CH0, MQ9_GAS_CO, 0, 0);

	DISABLE_5V_1V4;
	ENABLE_1V4;

	IRQEN = IRQ_TIMER;
	MAX_COUNT = 1 * 1000000;

	estado = 1;

	lectura_CO = (int)(ppm + 0.5f);
	_printf("Medicion de C0 en curso, por favor espere 60 seg.\n");
	_printf("Lectura CO: %d ppm \n", lectura_CO);
	_printf("Medicion de CH4 en curso, por favor espere 90 seg.\n");
}

void Lectura_CH4(void)
{
	float ppm;

	ppm = MQ9_GetValue(MCP3004_CH0, MQ9_GAS_CH4, 0, 0);

	DISABLE_5V_1V4;

	estado = 2;

	lectura_CH4 = (int)(ppm + 0.5f);
	_printf("Lectura CH4: %d ppm \n", lectura_CH4);
}

