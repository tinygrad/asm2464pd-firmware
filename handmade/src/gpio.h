#ifndef __GPIO_H__
#define __GPIO_H__

#include "types.h"
#include "registers.h"

#define GPIO_INPUT      0x00
#define GPIO_LOW        0x02
#define GPIO_HIGH       0x03

#define GPIO_NUM_MAX    27

static void gpio_set(uint8_t gpio_num, uint8_t mode) {
    REG_GPIO_CTRL(gpio_num) = mode;
}

static uint8_t gpio_read(uint8_t gpio_num) {
    return (REG_GPIO_INPUT(gpio_num) >> (gpio_num & 7)) & 1;
}

#define GPIO_LED_R      0
#define GPIO_LED_G      8
#define GPIO_LED_B      14

static void led_set_rgb(bool r, bool g, bool b) {
    gpio_set(GPIO_LED_R, r ? GPIO_LOW : GPIO_HIGH);
    gpio_set(GPIO_LED_G, g ? GPIO_LOW : GPIO_HIGH);
    gpio_set(GPIO_LED_B, b ? GPIO_LOW : GPIO_HIGH);
}

#endif
