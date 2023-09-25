#ifndef AS5600_H_INCLUDE
#define AS5600_H_INCLUDE

#include "driver/i2c.h"

/* Trailing zero hack */
#define TRAILING_ZERO(x) \
    ((x) == 0 ? 8 : ((x)&0x01 ? 0 : ((x)&0x02 ? 1 : ((x)&0x04 ? 2 : ((x)&0x08 ? 3 : ((x)&0x10 ? 4 : ((x)&0x20 ? 5 : ((x)&0x40 ? 6 : 7))))))))

/* Default I2C address of AS5600 */
#define AS5600_DEFAULT_ADDRESS 0x36

/* Raw angle registers */
#define AS5600_RAW_ANGLE_H 0x0C
#define AS5600_RAW_ANGLE_L 0x0D

/* Angle registers */
#define AS5600_ANGLE_H 0x0E
#define AS5600_ANGLE_L 0x0F

/* Status registers */
#define AS5600_STATUS 0x0B

/* Magnet detection bitmasks */
#define AS5600_STATUS_MH 0b00001000
#define AS5600_STATUS_ML 0b00010000
#define AS5600_STATUS_MD 0b00100000

/* Automatic Gain Control register */
#define AS5600_AGC 0x1A

/* Magniture registers */
#define AS5600_MAGNITUDE_H 0x1B
#define AS5600_MAGNITUDE_L 0x1C

/* Configuration registers */
#define AS5600_CONF_H 0x07
#define AS5600_CONF_L 0x08
#define AS5600_MANG_H 0x05
#define AS5600_MANG_L 0x06
#define AS5600_MPOS_H 0x03
#define AS5600_MPOS_L 0x04
#define AS5600_ZPOS_H 0x01
#define AS5600_ZPOS_L 0x02
#define AS5600_ZMCO 0x00

/* Configuration bitmasks */
#define AS5600_CONF_POWER_MODE 0b00000011
#define AS5600_CONF_HYSTERESIS 0b00001100
#define AS5600_CONF_OUT_STAGE 0b00110000
#define AS5600_CONF_PWM_FREQ 0b11000000
#define AS5600_CONF_SLOW_FILTER 0b00000011
#define AS5600_CONF_FAST_FILTER 0b00011100
#define AS5600_CONF_WATCHDOG 0b00100000

/* Angular conversion macros */
#define AS5600_RES 12
#define AS5600_FROM_ANGULAR(x) (((float)(1 << AS5600_RES) / 360.0f) * x)
#define AS5600_TO_ANGULAR(x) (x / ((float)(1 << AS5600_RES) / 360.0f))

/* Minimal angular range */
#define AS5600_MIN_ANGULAR 18

/* Burn register */
#define AS5600_BURN 0xFF

/* Burn commands */
#define AS5600_BURN_ANGLE 0x80
#define AS5600_BURN_SETTINGS 0x40

/* Angle burn max count */
#define AS5600_BURN_ANGLE_MAX_COUNT 3

/* OTP load commands */
#define AS5600_OTP_01 0x01
#define AS5600_OTP_02 0x11
#define AS5600_OTP_03 0x10

/* I2C operation timeout */
#define AS5600_MASTER_TIMEOUT_MS 1000

/*
 * AS5600 power mode types.
 */
typedef enum as5600_power_mode_type
{
    POWER_MODE_NOM,
    POWER_MODE_LPM1,
    POWER_MODE_LMP2,
    POWER_MODE_LMP3,
    POWER_MODE_MAX_NUM
} as5600_power_mode_type_e;

/*
 * AS5600 hysteresis types.
 */
typedef enum as5600_hysteresis_type
{
    HYPERESIS_OFF,
    HYSTERESIS_LSB1,
    HYSTERESIS_LSB2,
    HYSTERESIS_LSB3,
    HYSTERESIS_MAX_NUM
} as5600_hysteresis_type_e;

/*
 * AS5600 output stage types.
 */
typedef enum as5600_output_stage_type
{
    OUTPUT_STAGE_ANALOG_100,
    OUTPUT_STAGE_ANALOG_90,
    OUTPUT_STAGE_ANALOG_PWM,
    OUTPUT_STAGE_MAX_NUM
} as5600_output_stage_type_e;

/*
 * AS5600 PWM frequency types.
 */
typedef enum as5600_pwm_freq_type
{
    PWM_FREQ_115HZ,
    PWM_FREQ_230HZ,
    PWM_FREQ_460HZ,
    PWM_FREQ_920HZ,
    PWM_FREQ_MAX_NUM
} as5600_pwm_freq_type_e;

/*
 * AS5600 slow filter types.
 */
typedef enum as5600_slow_filter_type
{
    SLOW_FILTER_16X,
    SLOW_FILTER_8X,
    SLOW_FILTER_4X,
    SLOW_FILTER_2X,
    SLOW_FILTER_MAX_NUM
} as5600_slow_filter_type_e;

/*
 * AS5600 fast filter types.
 */
typedef enum as5600_fast_filter_type
{
    FAST_FILTER_SLOW_FILTER_ONLY,
    FAST_FILTER_LSB6,
    FAST_FILTER_LSB7,
    FAST_FILTER_LSB9,
    FAST_FILTER_LSB18,
    FAST_FILTER_LSB21,
    FAST_FILTER_LSB24,
    FAST_FILTER_LSB10,
    FAST_FILTER_MAX_NUM
} as5600_fast_filter_type_e;

/*
 * AS5600 watchdog types.
 */
typedef enum as5600_watchdog_type
{
    WATCHDOG_OFF,
    WATCHDOG_ON,
    WATCHDOG_MAX_NUM
} as5600_watchdog_type_e;

/*
 * Structure for burn angle command context.
 */
typedef struct as5600_burn_angle_context
{
    uint16_t start_position;
    uint16_t stop_position;
} as5600_burn_angle_context_t;

/*
 * Structure for burn settings command context.
 */
typedef struct as5600_burn_settings_context
{
    uint16_t max_angle;
} as5600_burn_settings_context_t;

/*
 * Structure for AS5600 sensor context.
 */
typedef struct as5600_handle
{
    i2c_port_t i2c_num;
    i2c_config_t i2c_config;
    uint8_t device_address;
} as5600_handle_t;

/*
 * Reads raw angel from sensor.
 * @param handle AS5600 handle
 * @param data buffer where raw angle is written
 * @return ESP_OK on success otherwise corresponding i2c_master_write_read_device error code
 */
esp_err_t as5600_get_raw_angle(as5600_handle_t *handle, uint16_t *data);

/*
 * Reads calibrated angel from sensor.
 * @param handle AS5600 handle
 * @param data buffer where calibrated angle is written
 * @return ESP_OK on success otherwise corresponding i2c_master_write_read_device error code
 */
esp_err_t as5600_get_angle(as5600_handle_t *handle, uint16_t *data);

/*
 * Reads from sensor if magnet is too strong.
 * @param handle AS5600 handle
 * @param data buffer where True is written if magnet is too strong otherwise False
 * @return ESP_OK on success otherwise corresponding i2c_master_write_read_device error code
 */
esp_err_t as5600_get_magnet_high(as5600_handle_t *handle, bool *data);

/*
 * Reads from sensor if magnet is too weak.
 * @param handle AS5600 handle
 * @param data buffer where True is written if magnet is too weak otherwise False
 * @return ESP_OK on success otherwise corresponding i2c_master_write_read_device error code
 */
esp_err_t as5600_get_magnet_low(as5600_handle_t *handle, bool *data);

/*
 * Reads from sensor if magnet is detected.
 * @param handle AS5600 handle
 * @param data buffer where True is written if magnet is detected otherwise False
 * @return ESP_OK on success otherwise corresponding i2c_master_write_read_device error code
 */
esp_err_t as5600_get_magnet_detect(as5600_handle_t *handle, bool *data);

/*
 * Reads AGC register data from sensor.
 * @param handle AS5600 handle
 * @param data buffer where AGC register data is written
 * @return ESP_OK on success otherwise corresponding i2c_master_write_read_device error code
 */
esp_err_t as5600_get_agc(as5600_handle_t *handle, uint8_t *data);

/*
 * Reads Magnitude register data from sensor.
 * @param handle AS5600 handle
 * @param data buffer where Magnitude register data is written
 * @return ESP_OK on success otherwise corresponding i2c_master_write_read_device error code
 */
esp_err_t as5600_get_magnitude(as5600_handle_t *handle, uint16_t *data);

/*
 * Reads power mode configuration from sensor.
 * @param handle AS5600 handle
 * @param power_mode buffer where corresponding power mode is written
 * @return ESP_OK on success otherwise corresponding i2c_master_write_read_device error code
 */
esp_err_t as5600_get_power_mode(as5600_handle_t *handle, as5600_power_mode_type_e *power_mode);

/*
 * Reads hysteresis configuration from sensor.
 * @param handle AS5600 handle
 * @param hysteresis buffer where corresponding hysteresis is written
 * @return ESP_OK on success otherwise corresponding i2c_master_write_read_device error code
 */
esp_err_t as5600_get_hysteresis(as5600_handle_t *handle, as5600_hysteresis_type_e *hysteresis);

/*
 * Reads output stage configuration from sensor.
 * @param handle AS5600 handle
 * @param output_stage buffer where corresponding output stage is written
 * @return ESP_OK on success otherwise corresponding i2c_master_write_read_device error code
 */
esp_err_t as5600_get_output_stage(as5600_handle_t *handle, as5600_output_stage_type_e *output_stage);

/*
 * Reads PWM frequency configuration from sensor.
 * @param handle AS5600 handle
 * @param pwm_freq buffer where corresponding PWM frequency stage is written
 * @return ESP_OK on success otherwise corresponding i2c_master_write_read_device error code
 */
esp_err_t as5600_get_pwm_freq(as5600_handle_t *handle, as5600_pwm_freq_type_e *pwm_freq);

/*
 * Reads slow filter configuration from sensor.
 * @param handle AS5600 handle
 * @param slow_filter buffer where corresponding slow filter stage is written
 * @return ESP_OK on success otherwise corresponding i2c_master_write_read_device error code
 */
esp_err_t as5600_get_slow_filter(as5600_handle_t *handle, as5600_slow_filter_type_e *slow_filter);

/*
 * Reads fast filter threshold configuration from sensor.
 * @param handle AS5600 handle
 * @param fast_filter buffer where corresponding fast filter threshold is written
 * @return ESP_OK on success otherwise corresponding i2c_master_write_read_device error code
 */
esp_err_t as5600_get_fast_filter(as5600_handle_t *handle, as5600_fast_filter_type_e *fast_filter);

/*
 * Reads watchdog configuration from sensor.
 * @param handle AS5600 handle
 * @param watchdog buffer where corresponding watchdog is written
 * @return ESP_OK on success otherwise corresponding i2c_master_write_read_device error code
 */
esp_err_t as5600_get_watchdog(as5600_handle_t *handle, as5600_watchdog_type_e *watchdog);

/*
 * Reads maximum angle register from sensor.
 * @param handle AS5600 handle
 * @param max_angle buffer where maximum angle is written
 * @return ESP_OK on success otherwise corresponding i2c_master_write_read_device error code
 */
esp_err_t as5600_get_mang(as5600_handle_t *handle, uint16_t *max_angle);

/*
 * Reads stop position register from sensor.
 * @param handle AS5600 handle
 * @param stop_position buffer where stop position is written
 * @return ESP_OK on success otherwise corresponding i2c_master_write_read_device error code
 */
esp_err_t as5600_get_mpos(as5600_handle_t *handle, uint16_t *stop_position);

/*
 * Reads start position register from sensor.
 * @param handle AS5600 handle
 * @param start_position buffer where start position is written
 * @return ESP_OK on success otherwise corresponding i2c_master_write_read_device error code
 */
esp_err_t as5600_get_zpos(as5600_handle_t *handle, uint16_t *start_position);

/*
 * Reads burn count from sensor.
 * @param handle AS5600 handle
 * @param zmco buffer where burn count is written
 * @return ESP_OK on success otherwise corresponding i2c_master_write_read_device error code
 */
esp_err_t as5600_get_zmco(as5600_handle_t *handle, uint8_t *zmco);

/*
 * Writes power mode configuration to sensor.
 * @param handle AS5600 handle
 * @param power_mode power mode setting
 * @return ESP_OK on success otherwise corresponding i2c_master_write_to_device error code
 */
esp_err_t as5600_set_power_mode(as5600_handle_t *handle, as5600_power_mode_type_e power_mode);

/*
 * Writes hysteresis configuration to sensor.
 * @param handle AS5600 handle
 * @param hysteresis hysteresis setting
 * @return ESP_OK on success otherwise corresponding i2c_master_write_to_device error code
 */
esp_err_t as5600_set_hysteresis(as5600_handle_t *handle, as5600_hysteresis_type_e hysteresis);

/*
 * Writes output stage configuration to sensor.
 * @param handle AS5600 handle
 * @param output_stage output stage setting
 * @return ESP_OK on success otherwise corresponding i2c_master_write_to_device error code
 */
esp_err_t as5600_set_output_stage(as5600_handle_t *handle, as5600_output_stage_type_e output_stage);

/*
 * Writes PWM frequency configuration to sensor.
 * @param handle AS5600 handle
 * @param pwm_freq PWM frequency setting
 * @return ESP_OK on success otherwise corresponding i2c_master_write_to_device error code
 */
esp_err_t as5600_set_pwm_freq(as5600_handle_t *handle, as5600_pwm_freq_type_e pwm_freq);

/*
 * Writes slow filter configuration to sensor.
 * @param handle AS5600 handle
 * @param slow_filter slow filter setting
 * @return ESP_OK on success otherwise corresponding i2c_master_write_to_device error code
 */
esp_err_t as5600_set_slow_filter(as5600_handle_t *handle, as5600_slow_filter_type_e slow_filter);

/*
 * Writes fast filter threshold configuration to sensor.
 * @param handle AS5600 handle
 * @param fast_filter fast filter threshold setting
 * @return ESP_OK on success otherwise corresponding i2c_master_write_to_device error code
 */
esp_err_t as5600_set_fast_filter(as5600_handle_t *handle, as5600_fast_filter_type_e fast_filter);

/*
 * Writes watchdog configuration to sensor.
 * @param handle AS5600 handle
 * @param fast_filter watchdog setting
 * @return ESP_OK on success otherwise corresponding i2c_master_write_to_device error code
 */
esp_err_t as5600_set_watchdog(as5600_handle_t *handle, as5600_watchdog_type_e watchdog);

/*
 * Writes maximum angle register to sensor.
 * @param handle AS5600 handle
 * @param max_angle maximum angle in AS5600 default resolution
 * @param context burn settings command context
 * @return ESP_OK on success otherwise corresponding i2c_master_write_to_device error code
 * @note this function only saves data in the register, you need commit changes using burn settings command
 */
esp_err_t as5600_set_mang(as5600_handle_t *handle, uint16_t max_angle, as5600_burn_settings_context_t *context);

/*
 * Writes stop position register to sensor.
 * @param handle AS5600 handle
 * @param stop_position stop position in AS5600 default resolution
 * @param context burn angle command context
 * @return ESP_OK on success otherwise corresponding i2c_master_write_to_device error code
 * @note this function only saves data in the register, you need commit changes using burn angle command
 */
esp_err_t as5600_set_mpos(as5600_handle_t *handle, uint16_t stop_position, as5600_burn_angle_context_t *context);

/*
 * Writes start position register to sensor.
 * @param handle AS5600 handle
 * @param start_position start position in AS5600 default resolution
 * @param context burn angle command context
 * @return ESP_OK on success otherwise corresponding i2c_master_write_to_device error code
 * @note this function only saves data in the register, you need commit changes using burn angle command
 */
esp_err_t as5600_set_zpos(as5600_handle_t *handle, uint16_t start_position, as5600_burn_angle_context_t *context);

/*
 * Burns angle (ZPOS, MPOS) to AS5600 sensor.
 * @param handle AS5600 handle
 * @param context burn angle command context
 * @return ESP_OK on success otherwise ESP_FAIL on wrong paramemetrs or corresponding i2c_master_write_to_device error code
 * @note This command can be executed AS5600_BURN_ANGLE_MAX_COUNT times on one sensor. ZMCO shows how many times angle was written already
 * @note Angle must be greater than AS5600_MIN_ANGULAR
 * @note By default, functionality is disabled, please configure AS5600_ENABLE_BURN_ANGLE in order to enable it
 * @note Please refer to AS5600 documentation for more details
 */
esp_err_t as5600_burn_angle(as5600_handle_t *handle, as5600_burn_angle_context_t *context);

/*
 * Burns settings (MANG, CONFIG) to AS5600 sensor.
 * @param handle AS5600 handle
 * @param context burn settings command context
 * @return ESP_OK on success otherwise ESP_FAIL on wrong paramemetrs or corresponding i2c_master_write_to_device error code
 * @note This command can be executed only one time when angle wasn't written
 * @note Angle must be greater than AS5600_MIN_ANGULAR
 * @note By default, functionality is disabled, please configure AS5600_ENABLE_BURN_SETTINGS in order to enable it
 * @note Please refer to AS5600 documentation for more details
 */
esp_err_t as5600_burn_settings(as5600_handle_t *handle, as5600_burn_settings_context_t *context);

/*
 * Loads the actual OTP content and verifies if burning was succesful.
 * @param handle AS5600 handle
 * @param command either AS5600_BURN_ANGLE or AS5600_BURN_SETTINGS
 * @param context burn command context
 * @return True if OTP content is same as given context otherwise False
 */
bool as5600_burn_verify(as5600_handle_t *handle, uint8_t command, void *context);

/*
 * Initializes I2C bus and install driver with given configuration.
 * @param handle AS5600 handle
 * @return ESP_OK if configured I2C bus and installed I2C driver otherwise ESP_ERR_INVALID_ARG or ESP_FAIL depending on internal errors
 */
esp_err_t as5600_init(as5600_handle_t *handle);

#endif