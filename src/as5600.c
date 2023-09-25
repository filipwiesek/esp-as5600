#include <stdio.h>

#include "sdkconfig.h"

#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c.h"

#include "as5600.h"

static const char *TAG = "AS5600";

esp_err_t as5600_read_byte(as5600_handle_t *handle, uint8_t reg_addr, uint8_t *data)
{
    return i2c_master_write_read_device(
        handle->i2c_num,
        handle->device_address,
        &reg_addr,
        1,
        data,
        1,
        AS5600_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t as5600_write_byte(as5600_handle_t *handle, uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_write_to_device(
        handle->i2c_num,
        handle->device_address,
        write_buf,
        sizeof(write_buf),
        AS5600_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

void as5600_get_raw_angle(as5600_handle_t *handle, uint16_t *data)
{
    uint8_t buf[2];
    as5600_read_byte(handle, AS5600_RAW_ANGLE_L, &buf[0]);
    as5600_read_byte(handle, AS5600_RAW_ANGLE_H, &buf[1]);

    *data = (buf[0] + (buf[1] << 8));
}

void as5600_get_angle(as5600_handle_t *handle, uint16_t *data)
{
    uint8_t buf[2];
    as5600_read_byte(handle, AS5600_ANGLE_L, &buf[0]);
    as5600_read_byte(handle, AS5600_ANGLE_H, &buf[1]);

    *data = (buf[0] + (buf[1] << 8));
}

void as5600_get_magnet_high(as5600_handle_t *handle, bool *data)
{
    uint8_t buf;
    as5600_read_byte(handle, AS5600_STATUS, &buf);
    *data = (buf & AS5600_STATUS_MH) > 1;
}

void as5600_get_magnet_low(as5600_handle_t *handle, bool *data)
{
    uint8_t buf;
    as5600_read_byte(handle, AS5600_STATUS, &buf);
    *data = (buf & AS5600_STATUS_ML) > 1;
}

void as5600_get_magnet_detect(as5600_handle_t *handle, bool *data)
{
    uint8_t buf;
    as5600_read_byte(handle, AS5600_STATUS, &buf);
    *data = (buf & AS5600_STATUS_MD) > 1;
}

void as5600_get_agc(as5600_handle_t *handle, uint8_t *data)
{
    as5600_read_byte(handle, AS5600_AGC, data);
}

void as5600_get_magnitude(as5600_handle_t *handle, uint16_t *data)
{
    uint8_t buf[2];
    as5600_read_byte(handle, AS5600_MAGNITUDE_L, &buf[0]);
    as5600_read_byte(handle, AS5600_MAGNITUDE_H, &buf[1]);

    *data = (buf[0] + (buf[1] << 8));
}

void as5600_get_power_mode(as5600_handle_t *handle, as5600_power_mode_type_e *power_mode)
{
    uint8_t buf;
    as5600_read_byte(handle, AS5600_CONF_L, &buf);
    *power_mode = (as5600_power_mode_type_e)((buf & AS5600_CONF_POWER_MODE) >> TRAILING_ZERO(AS5600_CONF_POWER_MODE));
    assert(*power_mode < POWER_MODE_MAX_NUM);
}

void as5600_get_hysteresis(as5600_handle_t *handle, as5600_hysteresis_type_e *hysteresis)
{
    uint8_t buf;
    as5600_read_byte(handle, AS5600_CONF_L, &buf);
    *hysteresis = (as5600_hysteresis_type_e)((buf & AS5600_CONF_HYSTERESIS) >> TRAILING_ZERO(AS5600_CONF_HYSTERESIS));
    assert(*hysteresis < HYSTERESIS_MAX_NUM);
}

void as5600_get_output_stage(as5600_handle_t *handle, as5600_output_stage_type_e *output_stage)
{
    uint8_t buf;
    as5600_read_byte(handle, AS5600_CONF_L, &buf);

    *output_stage = (as5600_output_stage_type_e)((buf & AS5600_CONF_OUT_STAGE) >> TRAILING_ZERO(AS5600_CONF_OUT_STAGE));
    assert(*output_stage < OUTPUT_STAGE_MAX_NUM);
}

void as5600_get_pwm_freq(as5600_handle_t *handle, as5600_pwm_freq_type_e *pwm_freq)
{
    uint8_t buf;
    as5600_read_byte(handle, AS5600_CONF_L, &buf);
    *pwm_freq = (as5600_pwm_freq_type_e)((buf & AS5600_CONF_PWM_FREQ) >> TRAILING_ZERO(AS5600_CONF_PWM_FREQ));
    assert(*pwm_freq < PWM_FREQ_MAX_NUM);
}

void as5600_get_slow_filter(as5600_handle_t *handle, as5600_slow_filter_type_e *slow_filter)
{
    uint8_t buf;
    as5600_read_byte(handle, AS5600_CONF_H, &buf);
    *slow_filter = (as5600_slow_filter_type_e)((buf & AS5600_CONF_SLOW_FILTER) >> TRAILING_ZERO(AS5600_CONF_SLOW_FILTER));
    assert(*slow_filter < SLOW_FILTER_MAX_NUM);
}

void as5600_get_fast_filter(as5600_handle_t *handle, as5600_fast_filter_type_e *fast_filter)
{
    uint8_t buf;
    as5600_read_byte(handle, AS5600_CONF_H, &buf);
    *fast_filter = (as5600_fast_filter_type_e)((buf & AS5600_CONF_FAST_FILTER) >> TRAILING_ZERO(AS5600_CONF_FAST_FILTER));
    assert(*fast_filter < FAST_FILTER_MAX_NUM);
}

void as5600_get_watchdog(as5600_handle_t *handle, as5600_watchdog_type_e *watchdog)
{
    uint8_t buf;
    as5600_read_byte(handle, AS5600_CONF_H, &buf);
    *watchdog = (as5600_power_mode_type_e)((buf & AS5600_CONF_WATCHDOG) >> TRAILING_ZERO(AS5600_CONF_WATCHDOG));
    assert(*watchdog < WATCHDOG_MAX_NUM);
}

void as5600_get_mang(as5600_handle_t *handle, uint16_t *max_angle)
{
    uint8_t buf[2];
    as5600_read_byte(handle, AS5600_MANG_L, &buf[0]);
    as5600_read_byte(handle, AS5600_MANG_H, &buf[1]);

    *max_angle = (buf[0] + (buf[1] << 8));
}

void as5600_get_mpos(as5600_handle_t *handle, uint16_t *stop_position)
{
    uint8_t buf[2];
    as5600_read_byte(handle, AS5600_MPOS_L, &buf[0]);
    as5600_read_byte(handle, AS5600_MPOS_H, &buf[1]);

    *stop_position = (buf[0] + (buf[1] << 8));
}

void as5600_get_zpos(as5600_handle_t *handle, uint16_t *start_position)
{
    uint8_t buf[2];
    as5600_read_byte(handle, AS5600_ZPOS_L, &buf[0]);
    as5600_read_byte(handle, AS5600_ZPOS_H, &buf[1]);

    *start_position = (buf[0] + (buf[1] << 8));
}

void as5600_get_zmco(as5600_handle_t *handle, uint8_t *zmco)
{
    as5600_read_byte(handle, AS5600_ZMCO, zmco);
}

bool as5600_set_power_mode(as5600_handle_t *handle, as5600_power_mode_type_e power_mode)
{
    if (power_mode >= POWER_MODE_MAX_NUM)
        return false;

    uint8_t buf;
    uint8_t write_buf = ((uint8_t)power_mode) << TRAILING_ZERO(AS5600_CONF_POWER_MODE);

    as5600_read_byte(handle, AS5600_CONF_L, &buf);

    buf ^= (buf ^ write_buf) & AS5600_CONF_POWER_MODE;

    return as5600_write_byte(handle, AS5600_CONF_L, buf) == ESP_OK;
}

bool as5600_set_hysteresis(as5600_handle_t *handle, as5600_hysteresis_type_e hysteresis)
{
    if (hysteresis >= HYSTERESIS_MAX_NUM)
        return false;

    uint8_t buf;
    uint8_t write_buf = ((uint8_t)hysteresis) << TRAILING_ZERO(AS5600_CONF_HYSTERESIS);

    as5600_read_byte(handle, AS5600_CONF_L, &buf);

    buf ^= (buf ^ write_buf) & AS5600_CONF_HYSTERESIS;

    return as5600_write_byte(handle, AS5600_CONF_L, buf) == ESP_OK;
}

bool as5600_set_output_stage(as5600_handle_t *handle, as5600_output_stage_type_e output_stage)
{
    if (output_stage >= OUTPUT_STAGE_MAX_NUM)
        return false;

    uint8_t buf;
    uint8_t write_buf = ((uint8_t)output_stage) << TRAILING_ZERO(AS5600_CONF_OUT_STAGE);

    as5600_read_byte(handle, AS5600_CONF_L, &buf);

    buf ^= (buf ^ write_buf) & AS5600_CONF_OUT_STAGE;

    return as5600_write_byte(handle, AS5600_CONF_L, buf) == ESP_OK;
}

bool as5600_set_pwm_freq(as5600_handle_t *handle, as5600_pwm_freq_type_e pwm_freq)
{
    if (pwm_freq >= PWM_FREQ_MAX_NUM)
        return false;

    uint8_t buf;
    uint8_t write_buf = ((uint8_t)pwm_freq) << TRAILING_ZERO(AS5600_CONF_PWM_FREQ);

    as5600_read_byte(handle, AS5600_CONF_L, &buf);

    buf ^= (buf ^ write_buf) & AS5600_CONF_PWM_FREQ;

    return as5600_write_byte(handle, AS5600_CONF_L, buf) == ESP_OK;
}

bool as5600_set_slow_filter(as5600_handle_t *handle, as5600_slow_filter_type_e slow_filter)
{
    if (slow_filter >= SLOW_FILTER_MAX_NUM)
        return false;

    uint8_t buf;
    uint8_t write_buf = ((uint8_t)slow_filter) << TRAILING_ZERO(AS5600_CONF_SLOW_FILTER);

    as5600_read_byte(handle, AS5600_CONF_H, &buf);

    buf ^= (buf ^ write_buf) & AS5600_CONF_SLOW_FILTER;

    return as5600_write_byte(handle, AS5600_CONF_H, buf) == ESP_OK;
}

bool as5600_set_fast_filter(as5600_handle_t *handle, as5600_fast_filter_type_e fast_filter)
{
    if (fast_filter >= FAST_FILTER_MAX_NUM)
        return false;

    uint8_t buf;
    uint8_t write_buf = ((uint8_t)fast_filter) << TRAILING_ZERO(AS5600_CONF_FAST_FILTER);

    as5600_read_byte(handle, AS5600_CONF_H, &buf);

    buf ^= (buf ^ write_buf) & AS5600_CONF_FAST_FILTER;
    as5600_write_byte(handle, AS5600_CONF_H, buf);

    return as5600_write_byte(handle, AS5600_CONF_H, buf) == ESP_OK;
}

bool as5600_set_watchdog(as5600_handle_t *handle, as5600_watchdog_type_e watchdog)
{
    if (watchdog >= WATCHDOG_MAX_NUM)
        return false;

    uint8_t buf;
    uint8_t write_buf = ((uint8_t)watchdog) << TRAILING_ZERO(AS5600_CONF_WATCHDOG);

    as5600_read_byte(handle, AS5600_CONF_H, &buf);

    buf ^= (buf ^ write_buf) & AS5600_CONF_WATCHDOG;
    as5600_write_byte(handle, AS5600_CONF_H, buf);

    return as5600_write_byte(handle, AS5600_CONF_H, buf) == ESP_OK;
}

bool as5600_set_mang(as5600_handle_t *handle, uint16_t max_angle, as5600_burn_settings_context_t *context)
{
    esp_err_t err;
    err = as5600_write_byte(handle, AS5600_MANG_L, max_angle & 0xFF);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "as5600_set_mang: failed to write mang.");
        return err;
    }

    err = as5600_write_byte(handle, AS5600_MANG_H, max_angle >> 8);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "as5600_set_mang: failed to write mang.");
        return err;
    }

    context->max_angle = max_angle;

    return true;
}

bool as5600_set_mpos(as5600_handle_t *handle, uint16_t stop_position, as5600_burn_angle_context_t *context)
{
    esp_err_t err;
    err = as5600_write_byte(handle, AS5600_MPOS_L, stop_position & 0xFF);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "as5600_set_mpos: failed to write mang.");
        return false;
    }

    err = as5600_write_byte(handle, AS5600_MPOS_H, stop_position >> 8);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "as5600_set_mpos: failed to write mang.");
        return false;
    }

    if (context)
        context->stop_position = stop_position;

    return true;
}

bool as5600_set_zpos(as5600_handle_t *handle, uint16_t start_position, as5600_burn_angle_context_t *context)
{
    esp_err_t err;
    err = as5600_write_byte(handle, AS5600_ZPOS_L, start_position & 0xFF);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "as5600_set_zpos: failed to write mang.");
        return false;
    }

    err = as5600_write_byte(handle, AS5600_ZPOS_H, start_position >> 8);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "as5600_set_zpos: failed to write mang.");
        return false;
    }

    if (context)
        context->start_position = start_position;

    return true;
}

bool as5600_burn_angle(as5600_handle_t *handle, as5600_burn_angle_context_t *context)
{
#ifndef CONFIG_AS5600_ENABLE_BURN_ANGLE
    ESP_LOGE(TAG, "as5600_burn_angle: function is disabled. Please configure AS5600_ENABLE_BURN_ANGLE to enable it. Use at your own risk!");
    return ESP_FAIL;
#endif

    uint8_t zmco;

    as5600_get_zmco(handle, &zmco);

    float angular_angle = AS5600_TO_ANGULAR(context->stop_position - context->start_position);
    if ((uint16_t)angular_angle < AS5600_MIN_ANGULAR)
    {
        ESP_LOGE(TAG, "as5600_burn_angle: failed to burn angle - angle too small (given - %f, min - %f)",
                 angular_angle, (float)AS5600_MIN_ANGULAR);
        return ESP_FAIL;
    }

    if (zmco >= AS5600_BURN_ANGLE_MAX_COUNT)
    {
        ESP_LOGE(TAG, "as5600_burn_angle: failed to burn angle - angle burn count exceeded (maximum - %d)",
                 AS5600_BURN_ANGLE_MAX_COUNT);
        return ESP_FAIL;
    }

    return as5600_write_byte(handle, AS5600_BURN, AS5600_BURN_ANGLE);
}

bool as5600_burn_settings(as5600_handle_t *handle, as5600_burn_settings_context_t *context)
{
#ifndef CONFIG_AS5600_ENABLE_BURN_SETTINGS
    ESP_LOGE(TAG, "as5600_burn_settings: function is disabled. Please configure AS5600_ENABLE_BURN_SETTINGS to enable it. Use at your own risk!");
    return ESP_FAIL;
#endif

    uint8_t zmco;

    as5600_get_zmco(handle, &zmco);

    float angular_angle = AS5600_TO_ANGULAR(context->max_angle);
    if ((uint16_t)angular_angle < AS5600_MIN_ANGULAR)
    {
        ESP_LOGE(TAG, "as5600_burn_settings: failed to burn settings - angle too small (given - %f, min - %f)",
                 angular_angle, (float)AS5600_MIN_ANGULAR);
        return ESP_FAIL;
    }

    if (zmco)
    {
        ESP_LOGE(TAG, "as5600_burn_settings: failed to burn settings - settings of angle already burnt");
        return ESP_FAIL;
    }

    return as5600_write_byte(handle, AS5600_BURN, AS5600_BURN_SETTINGS);
}

bool as5600_burn_verify(as5600_handle_t *handle, uint8_t command, void *context)
{
    if (
        as5600_write_byte(handle, AS5600_BURN, AS5600_OTP_01) != ESP_OK || as5600_write_byte(handle, AS5600_BURN, AS5600_OTP_02) != ESP_OK || as5600_write_byte(handle, AS5600_BURN, AS5600_OTP_03) != ESP_OK)
    {
        ESP_LOGE(TAG, "as5600_burn_verify: failed to load OTP content");
        return false;
    }

    switch (command)
    {
    case AS5600_BURN_ANGLE:
    {
        as5600_burn_angle_context_t *burn_angle_context = (as5600_burn_angle_context_t *)context;
        uint16_t start_position;
        uint16_t stop_position;

        as5600_get_zpos(handle, &start_position);
        as5600_get_mpos(handle, &stop_position);

        if (
            burn_angle_context->start_position == start_position && burn_angle_context->stop_position == stop_position)
            return true;

        return false;
    }
    case AS5600_BURN_SETTINGS:
    {
        as5600_burn_settings_context_t *burn_settings_context = (as5600_burn_settings_context_t *)context;
        uint16_t max_angle;

        as5600_get_mang(handle, &max_angle);

        if (burn_settings_context->max_angle == max_angle)
            return true;

        return false;
    }
    default:
    {
        ESP_LOGE(TAG, "as5600_burn_verify: unknown burn command (0x%X) to be verified!", command);
        return false;
    }
    }
}

esp_err_t as5600_init(as5600_handle_t *handle)
{
    esp_err_t err;
    err = i2c_param_config(handle->i2c_num, &handle->i2c_config);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "as5600_init: failed to configure I2C bus");
        return err;
    }

    err = i2c_driver_install(
        handle->i2c_num,
        handle->i2c_config.mode,
        0, 0, 0);

    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "as5600_init: sucessfully created I2C master");
        return err;
    }

    return ESP_OK;
}
