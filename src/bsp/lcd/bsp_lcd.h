/********************************************************************************************************//**
* @file bsp_lcd.h
*
* @brief Header file containing the prototypes of the APIs for BSP LCD.
*
* Public Functions:
*       - void bsp_lcd_init(void)
*/

#ifndef BSP_LCD_H
#define BSP_LCD_H

#include "stm32f429xx.h"

typedef struct
{
    SPI_TypeDef* spi;
    GPIO_TypeDef* gpio_port_resx;
    uint8_t gpio_num_resx;
    GPIO_TypeDef* gpio_port_csx;
    uint8_t gpio_num_csx;
    GPIO_TypeDef* gpio_port_dcx;
    uint8_t gpio_num_dcx;
}bsp_lcd_config_t;

/**
 * @brief Function for initializing the LCD.
 * @return void.
 */
void bsp_lcd_init(bsp_lcd_config_t* lcd_config);

#endif /* BSP_LCD_H */