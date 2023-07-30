#include "bsp_gpio.h"
#include "hal_gpio.h"

#define NUMBER_USED_PINS    5

hal_gpio_config_t gpio_list[NUMBER_USED_PINS] = {
    /* Configure RESX */
    {.gpio_port = GPIOA, .gpio_number = 7, .moder = GPIO_MODER_OUTPUT, .otyper = GPIO_OTYPER_PUSH_PULL, .ospeedr = GPIO_OSPEEDR_HIGH, .afr = 0, .odr = GPIO_ODR_HIGH},
    /* Configure CSX */
    {.gpio_port = GPIOC, .gpio_number = 2, .moder = GPIO_MODER_OUTPUT, .otyper = GPIO_OTYPER_PUSH_PULL, .ospeedr = GPIO_OSPEEDR_HIGH, .afr = 0, .odr = GPIO_ODR_HIGH},
    /* Configure DCX */
    {.gpio_port = GPIOD, .gpio_number = 13, .moder = GPIO_MODER_OUTPUT, .otyper = GPIO_OTYPER_PUSH_PULL, .ospeedr = GPIO_OSPEEDR_HIGH, .afr = 0, .odr = GPIO_ODR_HIGH},
    /* Configure SPI CLK */
    {.gpio_port = GPIOF, .gpio_number = 7, .moder = GPIO_MODER_AF, .otyper = GPIO_OTYPER_PUSH_PULL, .ospeedr = GPIO_OSPEEDR_HIGH, .afr = 5, .odr = GPIO_ODR_LOW},
    /* Configure SPI SDA */
    {.gpio_port = GPIOF, .gpio_number = 9, .moder = GPIO_MODER_AF, .otyper = GPIO_OTYPER_PUSH_PULL, .ospeedr = GPIO_OSPEEDR_HIGH, .afr = 5, .odr = GPIO_ODR_LOW},
};

void bsp_gio_configure_gpios(void)
{
    for(int i = 0; i < NUMBER_USED_PINS; i++)
    {
        hal_gpio_init(&gpio_list[i]);
    }
}