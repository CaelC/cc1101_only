#ifndef __SPI__COMMUNICATION__H__
#define __SPI__COMMUNICATION__H__


#include <stdint.h>
#include "driver/gpio.h"


#ifndef SPI_CLOCK_MAX
    #define SPI_CLOCK_MAX   16000000
#endif

#define SPI_CLOCK_1M        500000
#define SPI_CLOCK_DEFAULT   8000000

#define CONFIG_SPI_MOSI_GPIO            GPIO_NUM_13
#define CONFIG_SPI_MISO_GPIO            GPIO_NUM_12
#define CONFIG_SPI_SCLK_GPIO            GPIO_NUM_14

#define CONFIG_SPI3_SCK_GPIO            GPIO_NUM_14
#define CONFIG_SPI3_MOSI_GPIO           GPIO_NUM_13     // Supposedly MOSI is GPIO_NUM_13 but ported code says 12


extern void hV_HAL_SPI3_define(gpio_num_t pinClock, gpio_num_t pinData);
extern void hV_HAL_SPI3_begin(void);
extern uint8_t hV_HAL_SPI3_read(void);
extern void hV_HAL_SPI3_write(uint8_t value);

extern uint8_t hV_HAL_SPI_transfer(uint8_t data);
extern uint8_t hV_HAL_SPI_transfer_strobe(uint8_t data);
extern void hV_HAL_SPI_begin(uint32_t speed);
extern void hV_HAL_SPI_end(void);

#endif 
