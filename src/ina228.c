/**
 * @file ina228.c
 * @author Adam Page (adam.page@ambiq.com)
 * @brief INA228 API Implementation
 * @version 0.1
 * @date 2024-10-01
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "arm_math.h"
#include "ina228.h"

#define INA228_I2CADDR_DEFAULT 0x40 ///< INA228 default i2c address
#define INA228_REG_CONFIG 0x00      ///< Configuration register
#define INA228_REG_ADCCFG 0x01      ///< ADC configuration register
#define INA228_REG_SHUNTCAL 0x02    ///< Shunt calibration register
#define INA228_REG_SHUNTTEMPCO 0x03 ///< Shunt temperature coefficient register
#define INA228_REG_VSHUNT 0x04      ///< Shunt voltage measurement register
#define INA228_REG_VBUS 0x05        ///< Bus voltage measurement register
#define INA228_REG_DIETEMP 0x06     ///< Temperature measurement register
#define INA228_REG_CURRENT 0x07     ///< Current result register
#define INA228_REG_POWER 0x08       ///< Power result register
#define INA228_REG_ENERGY 0x09      ///< Energy result register
#define INA228_REG_CHARGE 0x0A      ///< Charge result register
#define INA228_REG_DIAGALRT 0x0B    ///< Diagnostic flags and alert register
#define INA228_REG_SOVL 0x0C        ///< Shunt overvoltage threshold register
#define INA228_REG_SUVL 0x0D        ///< Shunt undervoltage threshold register
#define INA228_REG_BOVL 0x0E        ///< Bus overvoltage threshold register
#define INA228_REG_BUVL 0x0F        ///< Bus undervoltage threshold register
#define INA228_REG_TEMPLIMIT 0x10 ///< Temperature over-limit threshold register
#define INA228_REG_PWRLIMIT 0x10  ///< Power over-limit threshold register
#define INA228_REG_MFG_UID 0x3E   ///< Manufacturer ID register
#define INA228_REG_DVC_UID 0x3F   ///< Device ID and revision register
#define INA228_REG_MFG_VAL 0x5449
#define INA228_REG_DVC_VAL 0x228

uint32_t
ina228_initialize(ina228_context_t *ctx)
{
    ctx->_shunt_res = 0.1;
    ctx->_current_lsb = 0.0001;
    // Validate the device?
}

uint32_t
ina228_validate(ina228_context_t *ctx)
{
    uint16_t value;
    uint32_t rst;
    rst = ina228_read_register(ctx, INA228_REG_MFG_UID, &value, 0xFFFF);
    if (value != INA228_REG_MFG_VAL) {
        return 1;
    }
    rst = ina228_read_register(ctx, INA228_REG_DVC_UID, &value, 0xFFFF);
    if (value != INA228_REG_DVC_VAL) {
        return 1;
    }
    return 0;
}

static uint32_t
ina228_read_register(ina228_context_t *ctx, uint8_t reg, uint16_t *value, uint16_t mask)
{
    uint8_t i2cBuffer[2];
    uint32_t rst;
    i2cBuffer[0] = reg;
    rst = ctx->i2c_write_read(ctx->addr, i2cBuffer, 1, i2cBuffer, 2);
    *value = (i2cBuffer[0] << 8) | i2cBuffer[1];
    if (mask != 0xFFFF) {
        *value &= mask;
    }
    return rst;
}

static uin32_t
ina228_read_register_24(ina228_context_t *ctx, uint8_t reg, uint32_t *value)
{
    uint8_t i2cBuffer[3];
    uint32_t rst;
    i2cBuffer[0] = reg;
    rst = ctx->i2c_write_read(ctx->addr, i2cBuffer, 1, i2cBuffer, 3);
    *value = (i2cBuffer[0] << 16) | (i2cBuffer[1] << 8) | i2cBuffer[2];
    return rst;
}

static uint32_t
ina228_read_bits(ina228_context_t *ctx, uint8_t reg, uint16_t *value, uint16_t len, uint16_t offset)
{
    uint16_t mask = (1 << len) - 1;
    uint16_t regValue;
    uint32_t rst;
    rst = ina228_read_register(ctx, reg, &regValue, 0xFFFF);
    *value = (regValue >> offset) & mask;
    return rst;
}

static uint32_t
ina228_write_bits(ina228_context_t *ctx, uint8_t reg, uint16_t value, uint16_t len, uint16_t offset)
{
    uint16_t mask = (1 << len) - 1;
    uint16_t regValue;
    uint32_t rst;
    rst = ina228_read_register(ctx, reg, &regValue, 0xFFFF);
    regValue &= ~(mask << offset);
    regValue |= (value & mask) << offset;
    return ina228_write_register(ctx, reg, regValue, 0xFFFF);
}

static uin32_t
ina228_write_register(ina228_context_t *ctx, uint8_t reg, uint16_t value, uint16_t mask)
{
    uint16_t rdValue;
    uint8_t i2cBuffer[3];
    i2cBuffer[0] = reg;
    if (mask != 0xFFFF) {
        ina228_read_register(ctx, reg, &rdValue, ~mask);
        value = (rdValue & ~mask) | (value & mask);
    }
    i2cBuffer[1] = (value >> 8) & 0xFF;
    i2cBuffer[2] = value & 0xFF;
    return ctx->i2c_write(&(i2cBuffer[0]), 3, ctx->addr);
}


uint32_t
ina228_reset(ina228_context_t *ctx)
{
    return ina228_write_register(ctx, INA228_REG_CONFIG, 0xC000, 0xC000);
}


uint32_t
ina228_reset_accumulators(ina228_context_t *ctx)
{
    return ina228_write_register(ctx, INA228_REG_CONFIG, 0x4000, 0x4000);
}

uint32_t
ina228_set_shunt(ina228_context_t *ctx,  float32_t shunt_res, float32_t max_current)
{
    uint8_t scale;
    ina228_get_adc_range(ctx, &scale);

    ctx->_shunt_res = shunt_res;
    ctx->_current_lsb = max_current / (float32_t)(1UL << 19);

    float32_t shunt_cal = 13107.2 * 1000000.0 * ctx->_shunt_res * ctx->_current_lsb * scale;
    return ina228_write_register(ctx, INA228_REG_SHUNTCAL, (uint16_t)shunt_cal, 0x7FFF);
}

uint32_t
ina228_set_adc_range(ina228_context_t *ctx, uint16_t adc_range)
{
    return ina228_write_bits(ctx, INA228_REG_ADCCFG, adc_range, 1, 4);
}

uint32_t
ina228_get_adc_range(ina228_context_t *ctx, uint16_t *adc_range)
{
    return ina228_read_bits(ctx, INA228_REG_ADCCFG, adc_range, 1, 4);
}

uint32_t
ina228_read_die_temp(ina228_context_t *ctx, float32_t *temp)
{
    uint16_t uvalue;
    int16_t value;
    uint32_t rst;
    rst = ina228_read_register(ctx, INA228_REG_DIETEMP, &uvalue, 0xFFFF);
    // Coerce to signed
    value = (int16_t)uvalue;
    *temp = (float32_t)value * 7.8125 / 1000.0;
    return rst;
}

uint32_t
ina228_read_current(ina228_context_t *ctx, float32_t *current)
{

    uint32_t rst;
    int32_t value;
    rst = ina228_read_register_24(ctx, INA228_REG_CURRENT, &value);
    if (value & 0x800000) {
        value |= 0xFF000000;
    }
    *current = (float32_t)value / 16.0 * ctx->_current_lsb * 1000.0;
}


uint32_t
ina228_read_bus_voltage(ina228_context_t *ctx, float32_t *bus_voltage)
{
    uint32_t rst;
    uint32_t value;
    rst = ina228_read_register_24(ctx, INA228_REG_VBUS, &value);
    // Never negative
    *bus_voltage = (float32_t)((uint32_t)value >> 4) * 195.3125 / 1000.0;
}

uint32_t
ina228_read_shunt_voltage(ina228_context_t *ctx, float32_t *shunt_voltage)
{
    uint32_t rst;
    uint8_t value8;
    int32_t value;

    rst = ina228_get_adc_range(ctx, &value8);
    float32_t scale = value8 ? 78.125 : 312.5;

    rst = ina228_read_register_24(ctx, INA228_REG_VSHUNT, &value);
    if (value & 0x800000) {
        value |= 0xFF000000;
    }

    *shunt_voltage = (float32_t)value / 16.0 * scale / 1000000.0;
    return rst;
}

uint32_t
ina228_read_power(ina228_context_t *ctx, float32_t *power)
{
    uint32_t rst;
    uint32_t value;
    rst = ina228_read_register_24(ctx, INA228_REG_POWER, &value);
    *power = (float32_t)value * 3.2 * _current_lsb * 1000;
    return rst;
}

uint32_t
ina228_read_energy(ina228_context_t *ctx, float32_t *energy) {
    uint32_t rst;
    uint8_t buff[5];
    buff[0] = INA228_REG_ENERGY;
    rst = ctx->i2c_write_read(ctx->addr, buff, 1, buff, 5);
    float32_t e = 0;
    for (int i = 0; i < 5; i++) {
        e *= 256;
        e += buff[i];
    }
    *energy = e * 16 * 3.2 * ctx->_current_lsb;
    return rst;
}

uint32_t
ina228_set_mode(ina228_context_t *ctx, ina228_meas_mode_t mode)
{
    return ina228_write_bits(ctx, INA228_REG_ADCCFG, mode, 4, 12);
}

uint32_t
ina228_get_mode(ina228_context_t *ctx, ina228_meas_mode_t *mode)
{
    uint16_t value;
    uint32_t rst;
    rst = ina228_read_bits(ctx, INA228_REG_ADCCFG, &value, 4, 12);
    *mode = (ina228_meas_mode_t)value;
    return rst;
}

uint32_t
ina228_conversion_ready(ina228_context_t *ctx, bool *ready)
{
    uint16_t value;
    uint32_t rst;
    rst = ina228_read_bits(ctx, INA228_REG_DIAGALRT, &value, 1, 0);
    *ready = value ? true : false;
    return rst;
}

uint32_t
ina228_alert_functions(ina228_context_t *ctx, uint16_t *functions)
{
    uint16_t value;
    uint32_t rst;
    rst = ina228_read_bits(ctx, INA228_REG_DIAGALRT, &value, 12, 0);
    *functions = value;
}

uint32_t
ina228_get_alert_latch(ina228_context_t *ctx, ina228_alert_latch_t *latch)
{
    uint16_t value;
    uint32_t rst;
    rst = ina228_read_bits(ctx, INA228_REG_DIAGALRT, &value, 1, 15);
    *latch = value ? INA228_ALERT_LATCH_ENABLED : INA228_ALERT_LATCH_TRANSPARENT;
}

uint32_t
ina228_set_alert_latch(ina228_context_t *ctx, ina228_alert_latch_t latch)
{
    return ina228_write_bits(ctx, INA228_REG_DIAGALRT, latch, 1, 15);
}

uint32_t
ina228_get_alert_polarity(ina228_context_t *ctx, ina228_alert_polarity_t *polarity)
{
    uint16_t value;
    uint32_t rst;
    rst = ina228_read_bits(ctx, INA228_REG_DIAGALRT, &value, 1, 12);
    *polarity = value ? INA228_ALERT_POLARITY_INVERTED : INA228_ALERT_POLARITY_NORMAL;
    return rst;
}

uint32_t
ina228_set_alert_polarity(ina228_context_t *ctx, ina228_alert_polarity_t polarity)
{
    return ina228_write_bits(ctx, INA228_REG_DIAGALRT, polarity, 1, 12);
}

uint32_t
ina228_get_current_conversion_time(ina228_context_t *ctx, ina228_conversion_time_t *time)
{
    uint16_t value;
    uint32_t rst;
    rst = ina228_read_bits(ctx, INA228_REG_ADCCFG, &value, 3, 6);
    *time = (ina228_conversion_time_t)value;
    return rst;

}

uint32_t
ina228_set_current_conversion_time(ina228_context_t *ctx, ina228_conversion_time_t time)
{
    return ina228_write_bits(ctx, INA228_REG_ADCCFG, time, 3, 6);
}

uint32_t
ina228_get_voltage_conversion_time(ina228_context_t *ctx, ina228_conversion_time_t *time)
{
    uint16_t value;
    uint32_t rst;
    rst = ina228_read_bits(ctx, INA228_REG_ADCCFG, &value, 3, 9);
    *time = (ina228_conversion_time_t)value;
    return rst;
}

uint32_t
ina228_set_voltage_conversion_time(ina228_context_t *ctx, ina228_conversion_time_t time)
{
    return ina228_write_bits(ctx, INA228_REG_ADCCFG, time, 3, 9);
}

uint32_t
ina228_get_temperature_conversion_time(ina228_context_t *ctx, ina228_conversion_time_t *time)
{
    uint16_t value;
    uint32_t rst;
    rst = ina228_read_bits(ctx, INA228_REG_ADCCFG, &value, 3, 3);
    *time = (ina228_conversion_time_t)value;
    return rst;
}

uint32_t
ina228_set_temperature_conversion_time(ina228_context_t *ctx, ina228_conversion_time_t time)
{
    // Bits 3-5
    return ina228_write_bits(ctx, INA228_REG_ADCCFG, time, 3, 3);
}

uint32_t
ina228_get_averaging_count(ina228_context_t *ctx, ina228_avg_count_t *count)
{
    uint16_t value;
    uint32_t rst;
    rst = ina228_read_bits(ctx, INA228_REG_ADCCFG, &value, 3, 0);
    *count = (ina228_avg_count_t)(value);
    return rst;

}

uint32_t
ina228_set_averaging_count(ina228_context_t *ctx, ina228_avg_count_t count)
{
    return ina228_write_bits(ctx, INA228_REG_ADCCFG, count, 3, 0);
}
