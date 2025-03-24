#include "spi_communication.h"


#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_rom_sys.h"
#include "esp_log.h" // For ESP-IDF logging


static SemaphoreHandle_t mutex = NULL;

// SPI handle
static spi_device_handle_t spi_handle;

// Structure for bit-banged SPI
typedef struct {
    gpio_num_t pinClock;
    gpio_num_t pinData;
} h_pinSPI3_t;

static h_pinSPI3_t h_pinSPI3;
bool flagSPI = false;


// Bit-banged SPI functions
void hV_HAL_SPI3_define(gpio_num_t pinClock, gpio_num_t pinData) {
    if(mutex == NULL) {
        mutex = xSemaphoreCreateMutex();
    }

    xSemaphoreTake(mutex, portMAX_DELAY);

    h_pinSPI3.pinClock = pinClock;
    h_pinSPI3.pinData = pinData;
    
    // Configure pins
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    // Configure clock pin
    io_conf.pin_bit_mask = (1ULL << pinClock);
    gpio_config(&io_conf);

    // Configure data pin (will be reconfigured as needed)
    io_conf.pin_bit_mask = (1ULL << pinData);
    gpio_config(&io_conf);

    flagSPI = false;

    xSemaphoreGive(mutex);
    
}

void hV_HAL_SPI3_begin(void) {
    // Use your defined GPIO pins here
    hV_HAL_SPI3_define(CONFIG_SPI3_SCK_GPIO, CONFIG_SPI3_MOSI_GPIO);
}

uint8_t hV_HAL_SPI3_read(void) {
    uint8_t value = 0;

    xSemaphoreTake(mutex, portMAX_DELAY);

    if(flagSPI == true) {
        xSemaphoreGive(mutex);
        hV_HAL_SPI3_begin();
        xSemaphoreTake(mutex, portMAX_DELAY);
    }

    gpio_set_direction(h_pinSPI3.pinClock, GPIO_MODE_OUTPUT);
    gpio_set_direction(h_pinSPI3.pinData, GPIO_MODE_INPUT);

    for (uint8_t i = 0; i < 8; ++i) {
        gpio_set_level(h_pinSPI3.pinClock, 1);
        esp_rom_delay_us(1);
        value |= gpio_get_level(h_pinSPI3.pinData) << (7 - i);
        gpio_set_level(h_pinSPI3.pinClock, 0);
        esp_rom_delay_us(1);
    }

    xSemaphoreGive(mutex);

    return value;
}

void hV_HAL_SPI3_write(uint8_t value) {
    xSemaphoreTake(mutex, portMAX_DELAY);

    if(flagSPI == true) {
        xSemaphoreGive(mutex);
        hV_HAL_SPI3_begin();
        xSemaphoreTake(mutex, portMAX_DELAY);
    }

    gpio_set_direction(h_pinSPI3.pinClock, GPIO_MODE_OUTPUT);
    gpio_set_direction(h_pinSPI3.pinData, GPIO_MODE_OUTPUT);

    for (uint8_t i = 0; i < 8; i++) {
        gpio_set_level(h_pinSPI3.pinData, !!(value & (1 << (7 - i))));
        esp_rom_delay_us(1);
        gpio_set_level(h_pinSPI3.pinClock, 1);
        esp_rom_delay_us(1);
        gpio_set_level(h_pinSPI3.pinClock, 0);
        esp_rom_delay_us(1);
    }

    xSemaphoreGive(mutex);
}

// normal SPI functions
uint8_t hV_HAL_SPI_transfer(uint8_t data) {
    uint8_t rx_data;

    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &data,
        .rx_buffer = &rx_data
    };

    // necessary because if the mutex is not initialized and an ISR is called, a reset will occur
    if(mutex == NULL) {
        mutex = xSemaphoreCreateMutex();
    }
    xSemaphoreTake(mutex, portMAX_DELAY);

    if(flagSPI == false) {
        xSemaphoreGive(mutex);
        hV_HAL_SPI_begin(SPI_CLOCK_1M);
        xSemaphoreTake(mutex, portMAX_DELAY);
    }

    spi_device_transmit(spi_handle, &t);

    xSemaphoreGive(mutex);

    return rx_data;
}

// normal SPI functions
uint8_t hV_HAL_SPI_transfer_strobe(uint8_t data) {
    uint8_t rx_data, dummy = 0x00, actual_result = 0x00;

    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &data,
        .rx_buffer = &rx_data
    };

    spi_transaction_t t_res = {
        .length = 8,
        .tx_buffer = &dummy,
        .rx_buffer = &actual_result
    };

    // necessary because if the mutex is not initialized and an ISR is called, a reset will occur
    if(mutex == NULL) {
        mutex = xSemaphoreCreateMutex();
    }
    xSemaphoreTake(mutex, portMAX_DELAY);

    if(flagSPI == false) {
        xSemaphoreGive(mutex);
        hV_HAL_SPI_begin(SPI_CLOCK_1M);
        xSemaphoreTake(mutex, portMAX_DELAY);
    }

    spi_device_transmit(spi_handle, &t);
    spi_device_transmit(spi_handle, &t_res);

    xSemaphoreGive(mutex);

    return actual_result;
}

void hV_HAL_SPI_begin(uint32_t speed)
{
    if(mutex == NULL) {
        mutex = xSemaphoreCreateMutex();
    }

    xSemaphoreTake(mutex, portMAX_DELAY);
    // double SPI initialization avoidance
    if (flagSPI != true) {
        spi_bus_config_t buscfg = {
            .mosi_io_num = CONFIG_SPI_MOSI_GPIO,  // Define these in your project config
            .miso_io_num = CONFIG_SPI_MISO_GPIO,
            .sclk_io_num = CONFIG_SPI_SCLK_GPIO,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 0  // Use default
        };

        spi_device_interface_config_t devcfg = {
            .clock_speed_hz = speed,
            .mode = 0,                    // SPI mode 0
            .spics_io_num = -1,
            .queue_size = 7,
            .flags = SPI_DEVICE_NO_DUMMY
        };

        // Initialize the SPI bus and device
        ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
        ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi_handle));

        flagSPI = true;

        xSemaphoreGive(mutex);
    }
}

void hV_HAL_SPI_end(void) {
    if(mutex == NULL) {
        mutex = xSemaphoreCreateMutex();
    }
    
    xSemaphoreTake(mutex, portMAX_DELAY);
    if (flagSPI == true) {
        spi_bus_remove_device(spi_handle);
        spi_bus_free(SPI2_HOST);
        flagSPI = false;
    }

    xSemaphoreGive(mutex);
}
