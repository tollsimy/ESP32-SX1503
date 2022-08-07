#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ESP32_SX1503.h"

uint8_t SX_SDA_PIN=18;
uint8_t SX_SCL_PIN=19;
uint8_t SX_I2C_PORT=0;
uint8_t SX_IRQ_PIN=10; //random number
uint8_t SX_RST_PIN=5; //random number


static const char *TAG = "Demo-SX1503";

void SX1503_task(){
    for(;;){
        ESP32_SX1503 SX1503={0};
        ESP_LOGI(TAG, "SX1503 task started");
        SX_init(&SX1503, SX_IRQ_PIN, SX_RST_PIN);

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

        SX_deinit();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{   
    xTaskCreate(&SX1503_task, "SX1503_task", 4096, NULL, 5, NULL);
}

