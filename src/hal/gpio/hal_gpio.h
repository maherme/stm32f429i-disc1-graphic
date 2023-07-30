/********************************************************************************************************//**
* @file hal_gpio.h
*
* @brief Header file containing the prototypes of the APIs for GPIO peripheral.
*
* Public Functions:
*/

#ifndef HAL_GPIO_H
#define HAL_GPIO_H

#include <stdint.h>
#include "stm32f429xx.h"

typedef enum
{
    GPIO_MODER_INPUT,
    GPIO_MODER_OUTPUT,
    GPIO_MODER_AF,
    GPIO_MODER_ANALOG
}hal_gpio_moder_t;

typedef enum
{
    GPIO_OTYPER_PUSH_PULL,
    GPIO_OTYPER_OPEN_DRAIN
}hal_gpio_otyper_t;

typedef enum
{
    GPIO_OSPEEDR_LOW,
    GPIO_OSPEEDR_MEDIUM,
    GPIO_OSPEEDR_HIGH,
    GPIO_OSPEEDR_VERY_HIGH
}hal_gpio_ospeedr_t;

typedef enum
{
    GPIO_ODR_LOW,
    GPIO_ODR_HIGH
}hal_gpio_odr_t;

typedef struct
{
    GPIO_TypeDef* gpio_port;
    uint8_t gpio_number;
    hal_gpio_moder_t moder;
    hal_gpio_otyper_t otyper;
    hal_gpio_ospeedr_t ospeedr;
    uint8_t afr;
    hal_gpio_odr_t odr;
}hal_gpio_config_t;

void hal_gpio_init(hal_gpio_config_t* gpio_config);
void hal_gpio_enable_port_clock(GPIO_TypeDef* gpio_port);
void hal_gpio_set_high(GPIO_TypeDef* gpio_port, uint8_t gpio_number);
void hal_gpio_set_low(GPIO_TypeDef* gpio_port, uint8_t gpio_number);

#endif  /* HAL_GPIO_H */