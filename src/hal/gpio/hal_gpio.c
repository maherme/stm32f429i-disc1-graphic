/************************************************************************************************//**
* @file hal_gpio.c
*
* @brief File containing the APIs for managing the GPIO peripheral.
**/

#include <stdint.h>
#include "hal_gpio.h"
#include "stm32f4xx.h"

/***************************************************************************************************/
/*                                       Public API Definitions                                    */
/***************************************************************************************************/

void hal_gpio_init(hal_gpio_config_t* gpio_config)
{
    GPIO_TypeDef* gpio_port = gpio_config->gpio_port;
    uint8_t gpio_number = gpio_config->gpio_number;

    /* Enable port clock */
    hal_gpio_enable_port_clock(gpio_port);

    /* Configure mode register */
    MODIFY_REG(gpio_port->MODER,
               0x3 << (gpio_number*2),
               (gpio_config->moder) << (gpio_number*2)
              );

    /* Configure otype register */
    MODIFY_REG(gpio_port->OTYPER,
               0x1 << gpio_number,
              (gpio_config->otyper) << gpio_number
              );

    /* Configure ospeed register */
    MODIFY_REG(gpio_port->OSPEEDR,
               0x3 << (gpio_number*2),
               (gpio_config->ospeedr) << (gpio_number*2)
              );

    /* Configure alternate functionality */
    if(gpio_number < 8)
    {
        MODIFY_REG(gpio_port->AFR[0],
                   0xF << (gpio_number*4),
                   (gpio_config->afr) << (gpio_number*4)
                  );
    }
    else
    {
         MODIFY_REG(gpio_port->AFR[1],
                   0xF << (gpio_number*4),
                   (gpio_config->afr) << ((gpio_number%8)*4)
                  );       
    }

    /* Configure output data register */
    if(gpio_config->odr == GPIO_ODR_HIGH)
    {
        hal_gpio_set_high(gpio_port, gpio_number);
    }
    else
    {
        hal_gpio_set_low(gpio_port, gpio_number);
    }
}

void hal_gpio_enable_port_clock(GPIO_TypeDef* gpio_port)
{
    switch((intptr_t)gpio_port)
    {
        case (intptr_t)GPIOA:
            SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);
            break;
        case (intptr_t)GPIOB:
            SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN);
            break;
        case (intptr_t)GPIOC:
            SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN);
            break;
        case (intptr_t)GPIOD:
            SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIODEN);
            break;
        case (intptr_t)GPIOE:
            SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOEEN);
            break;
        case (intptr_t)GPIOF:
            SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOFEN);
            break;
        case (intptr_t)GPIOG:
            SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOGEN);
            break;
        case (intptr_t)GPIOH:
            SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOHEN);
            break;
        case (intptr_t)GPIOI:
            SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOIEN);
            break;
        case (intptr_t)GPIOJ:
            SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOJEN);
            break;
        case (intptr_t)GPIOK:
            SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOKEN);
            break;
        default:
            break;
    }
}

void hal_gpio_set_high(GPIO_TypeDef* gpio_port, uint8_t gpio_number)
{
    SET_BIT(gpio_port->ODR, 0x1 << gpio_number);
}

void hal_gpio_set_low(GPIO_TypeDef* gpio_port, uint8_t gpio_number)
{
    CLEAR_BIT(gpio_port->ODR, 0x1 << gpio_number);
}