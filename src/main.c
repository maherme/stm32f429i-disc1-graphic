/************************************************************************************************//**
* @file main.c
*
* @brief File containing the main function.
**/

#include "bsp_gpio.h"
#include "bsp_lcd.h"

bsp_lcd_config_t lcd_config = {
    .spi = SPI5,
    .gpio_port_resx = GPIOA,
    .gpio_num_resx = 7,
    .gpio_port_csx = GPIOC,
    .gpio_num_csx = 2,
    .gpio_port_dcx = GPIOD,
    .gpio_num_dcx = 13,
};

/***************************************************************************************************/
/*                                       Main Function                                             */
/***************************************************************************************************/

int main(void)
{
    bsp_gio_configure_gpios();
    bsp_lcd_init(&lcd_config);

    for(;;){
    }
}