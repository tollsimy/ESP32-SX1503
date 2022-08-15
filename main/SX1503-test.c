#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ESP32_SX1503.h"

static const char *TAG = "Demo-SX1503";

#define SDA_PIN (27)
#define SCL_PIN (32)
#define I2C_PORT (0)
#define SX_RST_PIN (26)

I2C_CONF={
    .mode = I2C_MODE_MASTER;
    .sda_io_num = SDA_PIN;
    .scl_io_num = SCL_PIN;
    .sda_pullup_en = GPIO_PULLUP_DISABLE;     //disable if you have external pullup
    .scl_pullup_en = GPIO_PULLUP_DISABLE;
    .master.clk_speed = 400000;               //I2C Full Speed
}
void SX1503_task(){
    //Install I2C Driver
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &(I2C_CONF)));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));

    for(;;){
        ESP32_SX1503 SX1503={0};
        ESP_LOGI(TAG, "SX1503 task started");
        SX_init(&SX1503, SX_RST_PIN, I2C_PORT);

        SX_gpioMode(&SX1503, 10, SX_MODE_INPUT);
        SX_gpioPulldown(&SX1503, 10, true);
        ESP_LOGI(TAG, "SX Pin 10 value: %d", SX_digitalRead(&SX1503, 10));
        
        SX_enableInt(&SX1503, 10, SX_INT_BOTH, true);
        ESP_LOGI(TAG, "SX Interrupt occurred on pin 10: %d", SX_readInt(&SX1503, 10));

        SX_gpioMode(&SX1503, 0, SX_MODE_OUTPUT);
        SX_digitalWrite(&SX1503, 0, true);
        vTaskDelay(1000/portTICK_PERIOD_MS);
        SX_digitalWrite(&SX1503, 0, false);

        uint8_t SX_gpios[2];
        SX_fast_gpioRead(&SX1503, SX_gpios);
        ESP_LOGI(TAG, "SX_gpios values: %d %d", SX_gpios[0], SX_gpios[1]);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    ESP_ERROR_CHECK(i2c_driver_delete(I2C_PORT));
}

void app_main(void)
{   
    xTaskCreate(&SX1503_task, "SX1503_task", 4096, NULL, 5, NULL);
}

