# ESP-AS5600
Simple ESP-IDF library to connect ESP32 to AS5600 sensor.

## Overview
ESP-AS5600 allows ESP32 to communicate with AS5600 sensor. As there are multiple of I2C controllers on the board, you can use any of them. 

## Example
- Creating an AS5600 handle:
```c
    as5600_handle_t as_handle = {
        .i2c_num = I2C_NUM_0,
        .i2c_config = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = GPIO_NUM_19,
            .scl_io_num = GPIO_NUM_16,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = 400000,
        },
        .device_address = AS5600_DEFAULT_ADDRESS,
    };

    as5600_init(&as_handle);
```

- Reading raw angle:
```c
    uint16_t raw_angle;
    as5600_get_raw_angle(&as_handle, &raw_angle);

    ESP_LOGI(TAG, "as5600 reading: raw -> %f", AS5600_TO_ANGULAR(raw_angle));
```

- Burning settings:
```c
    // burning
    as5600_burn_settings_context_t context = {};
    as5600_set_mang(&as_handle, AS5600_FROM_ANGULAR(180), &context);

    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    as5600_burn_settings(&as_handle, &context);

    // verification
    vTaskDelay(100 / portTICK_PERIOD_MS);
    bool success = as5600_burn_verify(&as_handle, AS5600_BURN_SETTINGS, &context);
```

## Documentation
There is no separate document with explanation of interfaces. Header file is richly detailed, please refer to that.