/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "driver/gpio.h"

#include "cc1101.h"


static const char *TAG = "cc1101";

static CC1101_t cc1101 = {
    .CSNpin = GPIO_NUM_18,
    .GDO0pin = GPIO_NUM_4,
    .MISOpin = GPIO_NUM_15
};


void app_main(void)
{
    ESP_LOGI(TAG, "CC1101 being initialized");

    if(CC1101_begin(&cc1101, 433.2e6, false) == false) {
        // FIXME: Handle radio status not working
        ESP_LOGI(TAG, "CC1101 initialization failed");

        return;
    }

    ESP_LOGI(TAG, "CC1101 initialization done");

    CC1101_setRXstate(&cc1101);

    ESP_LOGI(TAG, "CC1101 set into Rx state. Rx Loop starting");

    uint8_t pktSize = 0, packet[64];

    while(1)
    {

        while(pktSize == 0) {
            pktSize = CC1101_getPacket(&cc1101, packet);
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        ESP_LOGI(TAG, "Packet received: %d bytes", pktSize);
        for(uint8_t i = 0; i < pktSize; i++) {
            ESP_LOGI(TAG, "Byte %d: 0x%02x", i, packet[i]);
        }

    }
}
