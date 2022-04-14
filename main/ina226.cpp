

#include "ina226.h"
#include <stdint.h>
#include <math.h>
#include "esp_system.h"

#define SCL 			GPIO_NUM_22
#define SDA				GPIO_NUM_21


static void i2c_master_init(int sda, int scl)
{
	static bool initialized = false;
	if (!initialized)
	{	
		int i2c_master_port = 0;
		i2c_config_t conf;
		conf.mode = I2C_MODE_MASTER;
		conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
		conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
		conf.master.clk_speed = 400000;
		conf.sda_io_num = sda;
		conf.scl_io_num = scl;
		conf.clk_flags = 0;
		i2c_param_config(i2c_master_port, &conf);
		if (i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0) == ESP_OK)
		{
			initialized = true;
		}
	}
}



bool INA226::begin(uint8_t address)
{
	i2c_master_init(SDA, SCL);
	inaAddress = address;
	return true;
}

bool INA226::configure(ina226_averages_t avg, ina226_busConvTime_t busConvTime, ina226_shuntConvTime_t shuntConvTime, ina226_mode_t mode)
{
	uint16_t config = 0;

	config |= (avg << 9 | busConvTime << 6 | shuntConvTime << 3 | mode);

	vBusMax = 36;
	vShuntMax = 0.08192f;

	writeRegister16(INA226_REG_CONFIG, config);

	return true;
}

bool INA226::calibrate(float rShuntValue, float iMaxExpected)
{
	uint16_t calibrationValue;
	rShunt = rShuntValue;

	float iMaxPossible, minimumLSB;

	iMaxPossible = vShuntMax / rShunt;

	minimumLSB = iMaxExpected / 32767;

	currentLSB = (uint16_t)(minimumLSB * 100000000);
	currentLSB /= 100000000;
	currentLSB /= 0.0001;
	currentLSB = ceil(currentLSB);
	currentLSB *= 0.0001;

	powerLSB = currentLSB * 25;

	calibrationValue = (uint16_t)((0.00512) / (currentLSB * rShunt));

	writeRegister16(INA226_REG_CALIBRATION, calibrationValue);

	return true;
}

float INA226::getMaxPossibleCurrent(void)
{
	return (vShuntMax / rShunt);
}

float INA226::getMaxCurrent(void)
{
	float maxCurrent = (currentLSB * 32767);
	float maxPossible = getMaxPossibleCurrent();

	if (maxCurrent > maxPossible)
	{
		return maxPossible;
	}
	else
	{
		return maxCurrent;
	}
}

float INA226::getMaxShuntVoltage(void)
{
	float maxVoltage = getMaxCurrent() * rShunt;

	if (maxVoltage >= vShuntMax)
	{
		return vShuntMax;
	}
	else
	{
		return maxVoltage;
	}
}

float INA226::getMaxPower(void)
{
	return (getMaxCurrent() * vBusMax);
}

float INA226::readBusPower(void)
{
	return (readRegister16(INA226_REG_POWER) * powerLSB);
}

float INA226::readShuntCurrent(void)
{
	return (readRegister16(INA226_REG_CURRENT) * currentLSB);
}

float INA226::readShuntVoltage(void)
{
	float voltage;

	voltage = readRegister16(INA226_REG_SHUNTVOLTAGE);

	return (voltage * 0.0000025);
}

float INA226::readBusVoltage(void)
{
	int16_t voltage;

	voltage = readRegister16(INA226_REG_BUSVOLTAGE);

	return (voltage * 0.00125);
}

ina226_averages_t INA226::getAverages(void)
{
	uint16_t value;

	value = readRegister16(INA226_REG_CONFIG);
	value &= 0b0000111000000000;
	value >>= 9;

	return (ina226_averages_t)value;
}

ina226_busConvTime_t INA226::getBusConversionTime(void)
{
	uint16_t value;

	value = readRegister16(INA226_REG_CONFIG);
	value &= 0b0000000111000000;
	value >>= 6;

	return (ina226_busConvTime_t)value;
}

ina226_shuntConvTime_t INA226::getShuntConversionTime(void)
{
	uint16_t value;

	value = readRegister16(INA226_REG_CONFIG);
	value &= 0b0000000000111000;
	value >>= 3;

	return (ina226_shuntConvTime_t)value;
}

ina226_mode_t INA226::getMode(void)
{
	uint16_t value;

	value = readRegister16(INA226_REG_CONFIG);
	value &= 0b0000000000000111;

	return (ina226_mode_t)value;
}

void INA226::setMaskEnable(uint16_t mask)
{
	writeRegister16(INA226_REG_MASKENABLE, mask);
}

uint16_t INA226::getMaskEnable(void)
{
	return readRegister16(INA226_REG_MASKENABLE);
}

void INA226::enableShuntOverLimitAlert(void)
{
	writeRegister16(INA226_REG_MASKENABLE, INA226_BIT_SOL);
}

void INA226::enableShuntUnderLimitAlert(void)
{
	writeRegister16(INA226_REG_MASKENABLE, INA226_BIT_SUL);
}

void INA226::enableBusOvertLimitAlert(void)
{
	writeRegister16(INA226_REG_MASKENABLE, INA226_BIT_BOL);
}

void INA226::enableBusUnderLimitAlert(void)
{
	writeRegister16(INA226_REG_MASKENABLE, INA226_BIT_BUL);
}

void INA226::enableOverPowerLimitAlert(void)
{
	writeRegister16(INA226_REG_MASKENABLE, INA226_BIT_POL);
}

void INA226::enableConversionReadyAlert(void)
{
	writeRegister16(INA226_REG_MASKENABLE, INA226_BIT_CNVR);
}

void INA226::setBusVoltageLimit(float voltage)
{
	uint16_t value = voltage / 0.00125;
	writeRegister16(INA226_REG_ALERTLIMIT, value);
}

void INA226::setShuntVoltageLimit(float voltage)
{
	uint16_t value = voltage * 25000;
	writeRegister16(INA226_REG_ALERTLIMIT, value);
}

void INA226::setPowerLimit(float watts)
{
	uint16_t value = watts / powerLSB;
	writeRegister16(INA226_REG_ALERTLIMIT, value);
}

void INA226::setAlertInvertedPolarity(bool inverted)
{
	uint16_t temp = getMaskEnable();

	if (inverted)
	{
		temp |= INA226_BIT_APOL;
	}
	else
	{
		temp &= ~INA226_BIT_APOL;
	}

	setMaskEnable(temp);
}

void INA226::setAlertLatch(bool latch)
{
	uint16_t temp = getMaskEnable();

	if (latch)
	{
		temp |= INA226_BIT_LEN;
	}
	else
	{
		temp &= ~INA226_BIT_LEN;
	}

	setMaskEnable(temp);
}

bool INA226::isMathOverflow(void)
{
	return ((getMaskEnable() & INA226_BIT_OVF) == INA226_BIT_OVF);
}

bool INA226::isAlert(void)
{
	return ((getMaskEnable() & INA226_BIT_AFF) == INA226_BIT_AFF);
}



int16_t INA226::readRegister16(uint8_t reg)
{
	uint8_t data[2];
	i2c_master_write_read_device(0, inaAddress, &reg, 1, data, 2, 1000 / portTICK_PERIOD_MS);
	return data[1] | (data[0] << 8);
}

void INA226::writeRegister16(uint8_t reg, uint16_t value)
{
	uint8_t data[3];
	data[0] = reg;
	data[1] = value;
	data[2] = value >> 8;
	i2c_master_write_to_device(0, inaAddress, data, sizeof(data), 1000 / portTICK_PERIOD_MS);
}
