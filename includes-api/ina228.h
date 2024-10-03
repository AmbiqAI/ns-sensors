/**
 * @file ina228.h
 * @author Adam Page (adam.page@ambiq.com)
 * @brief INA228 API Header file
 * @version 0.1
 * @date 2024-10-01
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "arm_math.h"

typedef uint32_t (*pfnI2cWriteRead)(uint16_t addr, const void *write_buf, size_t num_write, void *read_buf, size_t num_read);
typedef uint32_t (*pfnI2cRead)(const void *buf, uint32_t num_bytes, uint16_t addr);
typedef uint32_t (*pfnI2cWrite)(const void *buf, uint32_t num_bytes, uint16_t addr);


typedef struct {
  uint32_t addr;
  float _shunt_res;
  float _current_lsb;
  pfnI2cWriteRead i2c_write_read;
  pfnI2cRead i2c_read;
  pfnI2cWrite i2c_write;
} ina228_context_t;


/**
 * @brief Mode options.
 *
 * Allowed values for setMode.
 */
typedef enum {
  /**< SHUTDOWN: Minimize quiescient current and turn off current into the
  device inputs. Set another mode to exit shutown mode **/
  INA228_MODE_SHUTDOWN = 0x00,

  /**< Triggered bus voltage, single shot **/
  INA228_MODE_TRIG_BUS = 0x01,
  /**< Triggered shunt voltage, single shot **/
  INA228_MODE_TRIG_SHUNT = 0x02,
  /**< Triggered shunt voltage and bus voltage, single shot **/
  INA228_MODE_TRIG_BUS_SHUNT = 0x03,
  /**< Triggered temperature, single shot **/
  INA228_MODE_TRIG_TEMP = 0x04,
  /**< Triggered temperature and bus voltage, single shot **/
  INA228_MODE_TRIG_TEMP_BUS = 0x05,
  /**< Triggered temperature and shunt voltage, single shot **/
  INA228_MODE_TRIG_TEMP_SHUNT = 0x06,
  /**< Triggered bus voltage, shunt voltage and temperature, single shot **/
  INA228_MODE_TRIG_TEMP_BUS_SHUNT = 0x07,

  /**< Shutdown **/
  INA228_MODE_SHUTDOWN2 = 0x08,
  /**< Continuous bus voltage only **/
  INA228_MODE_CONT_BUS = 0x09,
  /**< Continuous shunt voltage only **/
  INA228_MODE_CONT_SHUNT = 0x0A,
  /**< Continuous shunt and bus voltage **/
  INA228_MODE_CONT_BUS_SHUNT = 0x0B,
  /**< Continuous temperature only **/
  INA228_MODE_CONT_TEMP = 0x0C,
  /**< Continuous bus voltage and temperature **/
  INA228_MODE_CONT_TEMP_BUS = 0x0D,
  /**< Continuous temperature and shunt voltage **/
  INA228_MODE_CONT_TEMP_SHUNT = 0x0E,
  /**< Continuous bus voltage, shunt voltage and temperature **/
  INA228_MODE_CONT_TEMP_BUS_SHUNT = 0x0F,

  /**< TRIGGERED: Trigger a one-shot measurement of temp, current and bus
  voltage. Set the TRIGGERED mode again to take a new measurement **/
  INA228_MODE_TRIGGERED = INA228_MODE_TRIG_TEMP_BUS_SHUNT,
  /**< CONTINUOUS: (Default) Continuously update the temp, current, bus
  voltage and power registers with new measurements **/
  INA228_MODE_CONTINUOUS = INA228_MODE_CONT_TEMP_BUS_SHUNT
} ina228_meas_mode_t;

/**
 * @brief Conversion Time options.
 *
 * Allowed values for setCurrentConversionTime and setVoltageConversionTime.
 */
typedef enum _conversion_time {
  INA228_TIME_50_us,   ///< Measurement time: 50us
  INA228_TIME_84_us,   ///< Measurement time: 84us
  INA228_TIME_150_us,  ///< Measurement time: 150us
  INA228_TIME_280_us,  ///< Measurement time: 280us
  INA228_TIME_540_us,  ///< Measurement time: 540us
  INA228_TIME_1052_us, ///< Measurement time: 1052us
  INA228_TIME_2074_us, ///< Measurement time: 2074us
  INA228_TIME_4120_us, ///< Measurement time: 4120us
} ina228_conversion_time_t;

/**
 * @brief Averaging Count options.
 *
 * Allowed values forsetAveragingCount.
 */
typedef enum {
  INA228_COUNT_1,    ///< Window size: 1 sample (Default)
  INA228_COUNT_4,    ///< Window size: 4 samples
  INA228_COUNT_16,   ///< Window size: 16 samples
  INA228_COUNT_64,   ///< Window size: 64 samples
  INA228_COUNT_128,  ///< Window size: 128 samples
  INA228_COUNT_256,  ///< Window size: 256 samples
  INA228_COUNT_512,  ///< Window size: 512 samples
  INA228_COUNT_1024, ///< Window size: 1024 samples
} ina228_avg_count_t;

/**
 * @brief Alert trigger options.
 *
 * Allowed values for setAlertType.
 */
typedef enum  {
  INA228_ALERT_CONVERSION_READY = 0x1, ///< Trigger on conversion ready
  INA228_ALERT_OVERPOWER = 0x2,        ///< Trigger on power over limit
  INA228_ALERT_UNDERVOLTAGE = 0x4,     ///< Trigger on bus voltage under limit
  INA228_ALERT_OVERVOLTAGE = 0x8,      ///< Trigger on bus voltage over limit
  INA228_ALERT_UNDERCURRENT = 0x10,    ///< Trigger on current under limit
  INA228_ALERT_OVERCURRENT = 0x20,     ///< Trigger on current over limit
  INA228_ALERT_NONE = 0x0,             ///< Do not trigger alert pin (Default)
} ina228_alert_type_t;

/**
 * @brief Alert pin polarity options.
 *
 * Allowed values for setAlertPolarity.
 */
typedef enum {
  INA228_ALERT_POLARITY_NORMAL = 0x0, ///< Active high open-collector (Default)
  INA228_ALERT_POLARITY_INVERTED = 0x1, ///< Active low open-collector
} ina228_alert_polarity_t;

/**
 * @brief Alert pin latch options.
 *
 * Allowed values for setAlertLatch.
 */
typedef enum {
  INA228_ALERT_LATCH_ENABLED = 0x1,     /**< Alert will latch until Mask/Enable
                                           register is read **/
  INA228_ALERT_LATCH_TRANSPARENT = 0x0, /**< Alert will reset when fault is
                                           cleared **/
} ina228_alert_latch_t;


uint32_t
ina228_initialize(ina228_context_t *ctx);

uint32_t
ina228_get_manufacturer_id(ina228_context_t *ctx, uint16_t *value);

uint32_t
ina228_get_device_id(ina228_context_t *ctx, uint16_t *value);

uint32_t
ina228_reset(ina228_context_t *ctx);

uint32_t
ina228_reset_accumulators(ina228_context_t *ctx);

uint32_t
ina228_set_shunt(ina228_context_t *ctx,  float shunt_res, float max_current);

uint32_t
ina228_set_adc_range(ina228_context_t *ctx, uint16_t adc_range);

uint32_t
ina228_get_adc_range(ina228_context_t *ctx, uint16_t *adc_range);

uint32_t
ina228_read_die_temp(ina228_context_t *ctx, float *temp);

uint32_t
ina228_read_current(ina228_context_t *ctx, float *current);

uint32_t
ina228_read_bus_voltage(ina228_context_t *ctx, float *bus_voltage);

uint32_t
ina228_read_shunt_voltage(ina228_context_t *ctx, float *shunt_voltage);

uint32_t
ina228_read_power(ina228_context_t *ctx, float *power);

uint32_t
ina228_read_energy(ina228_context_t *ctx, float *energy);

uint32_t
ina228_read_charge(ina228_context_t *ctx, float32_t *charge);

uint32_t
ina228_set_mode(ina228_context_t *ctx, ina228_meas_mode_t mode);

uint32_t
ina228_get_mode(ina228_context_t *ctx, ina228_meas_mode_t *mode);

uint32_t
ina228_conversion_ready(ina228_context_t *ctx, uint8_t *ready);

uint32_t
ina228_alert_functions(ina228_context_t *ctx, uint16_t *functions);

uint32_t
ina228_get_alert_latch(ina228_context_t *ctx, ina228_alert_latch_t *latch);

uint32_t
ina228_set_alert_latch(ina228_context_t *ctx, ina228_alert_latch_t latch);

uint32_t
ina228_get_alert_polarity(ina228_context_t *ctx, ina228_alert_polarity_t *polarity);

uint32_t
ina228_set_alert_polarity(ina228_context_t *ctx, ina228_alert_polarity_t polarity);

uint32_t
ina228_get_current_conversion_time(ina228_context_t *ctx, ina228_conversion_time_t *time);

uint32_t
ina228_set_current_conversion_time(ina228_context_t *ctx, ina228_conversion_time_t time);

uint32_t
ina228_get_voltage_conversion_time(ina228_context_t *ctx, ina228_conversion_time_t *time);

uint32_t
ina228_set_voltage_conversion_time(ina228_context_t *ctx, ina228_conversion_time_t time);

uint32_t
ina228_get_temperature_conversion_time(ina228_context_t *ctx, ina228_conversion_time_t *time);

uint32_t
ina228_set_temperature_conversion_time(ina228_context_t *ctx, ina228_conversion_time_t time);

uint32_t
ina228_get_averaging_count(ina228_context_t *ctx, ina228_avg_count_t *count);

uint32_t
ina228_set_averaging_count(ina228_context_t *ctx, ina228_avg_count_t count);
