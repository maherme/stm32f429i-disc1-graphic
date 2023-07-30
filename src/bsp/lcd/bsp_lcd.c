/************************************************************************************************//**
* @file bsp_lcd.c
*
* @brief File containing the board support package for LCD.
**/

#include "hal_spi.h"
#include "hal_gpio.h"
#include "bsp_lcd.h"
#include "stm32f429xx.h"
#include "stm32f4xx.h"
#include "ili9341_reg.h"

#define MADCTL_MY 0x80  /**< Bottom to top */
#define MADCTL_MX 0x40  /**< Right to left */
#define MADCTL_MV 0x20  /**< Reverse Mode */
#define MADCTL_ML 0x10  /**< LCD refresh Bottom to top */
#define MADCTL_RGB 0x00 /**< Red-Green-Blue pixel order */
#define MADCTL_BGR 0x08 /**< Blue-Green-Red pixel order */
#define MADCTL_MH 0x04  /**< LCD refresh right to left */

hal_spi_config_t spi_config = {.spi = SPI5,};

/***************************************************************************************************/
/*                                       Static Function Prototypes                                */
/***************************************************************************************************/

/**
 * @brief .
 * @return void.
 */
static void bsp_lcd_reset(bsp_lcd_config_t* lcd_config);

/**
 * @brief .
 * @return void.
 */
static void bsp_lcd_config(bsp_lcd_config_t* lcd_config);

void bsp_lcd_write_cmd(bsp_lcd_config_t* lcd_config, uint8_t cmd);
void bsp_lcd_write_data(bsp_lcd_config_t* lcd_config, uint8_t* buffer, uint32_t len);
void delay_50ms(void);

/***************************************************************************************************/
/*                                       Public API Definitions                                    */
/***************************************************************************************************/

void bsp_lcd_init(bsp_lcd_config_t* lcd_config)
{
    hal_spi_init(&spi_config);
    hal_spi_enable(lcd_config->spi);
    bsp_lcd_reset(lcd_config);
    bsp_lcd_config(lcd_config);
}

/***************************************************************************************************/
/*                                       Static Function Definitions                               */
/***************************************************************************************************/

static void bsp_lcd_reset(bsp_lcd_config_t* lcd_config)
{
    hal_gpio_set_low(lcd_config->gpio_port_resx, lcd_config->gpio_num_resx);
    delay_50ms();
    hal_gpio_set_high(lcd_config->gpio_port_resx, lcd_config->gpio_num_resx);
    delay_50ms();
}

static void bsp_lcd_config(bsp_lcd_config_t* lcd_config)
{
    uint8_t params[15] = {0};
    uint8_t m = MADCTL_MV | MADCTL_MY| MADCTL_BGR;

    bsp_lcd_write_cmd(lcd_config, ILI9341_SWRESET);
    bsp_lcd_write_cmd(lcd_config, ILI9341_POWERB);
    params[0] = 0x00;
    params[1] = 0xD9;
    params[2] = 0x30;
    bsp_lcd_write_data(lcd_config, params, 3);

    bsp_lcd_write_cmd(lcd_config, ILI9341_POWER_SEQ);
    params[0]= 0x64;
    params[1]= 0x03;
    params[2]= 0X12;
    params[3]= 0X81;
    bsp_lcd_write_data(lcd_config, params, 4);

    bsp_lcd_write_cmd(lcd_config, ILI9341_DTCA);
    params[0]= 0x85;
    params[1]= 0x10;
    params[2]= 0x7A;
    bsp_lcd_write_data(lcd_config, params, 3);

    bsp_lcd_write_cmd(lcd_config, ILI9341_POWERA);
    params[0]= 0x39;
    params[1]= 0x2C;
    params[2]= 0x00;
    params[3]= 0x34;
    params[4]= 0x02;
    bsp_lcd_write_data(lcd_config, params, 5);

    bsp_lcd_write_cmd(lcd_config, ILI9341_PRC);
    params[0]= 0x20;
    bsp_lcd_write_data(lcd_config, params, 1);

    bsp_lcd_write_cmd(lcd_config, ILI9341_DTCB);
    params[0]= 0x00;
    params[1]= 0x00;
    bsp_lcd_write_data(lcd_config, params, 2);

    bsp_lcd_write_cmd(lcd_config, ILI9341_POWER1);
    params[0]= 0x1B;
    bsp_lcd_write_data(lcd_config, params, 1);

    bsp_lcd_write_cmd(lcd_config, ILI9341_POWER2);
    params[0]= 0x12;
    bsp_lcd_write_data(lcd_config, params, 1);

    bsp_lcd_write_cmd(lcd_config, ILI9341_VCOM1);
    params[0]= 0x08;
    params[1]= 0x26;
    bsp_lcd_write_data(lcd_config, params, 2);

    bsp_lcd_write_cmd(lcd_config, ILI9341_VCOM2);
    params[0]= 0XB7;
    bsp_lcd_write_data(lcd_config, params, 1);

    bsp_lcd_write_cmd(lcd_config, ILI9341_MAC);    // Memory Access Control <Landscape setting>
    params[0]= m;
    bsp_lcd_write_data(lcd_config, params, 1);

    bsp_lcd_write_cmd(lcd_config, ILI9341_PIXEL_FORMAT);
    params[0]= 0x55; //select RGB565
    bsp_lcd_write_data(lcd_config, params, 1);

    bsp_lcd_write_cmd(lcd_config, ILI9341_FRMCTR1);
    params[0]= 0x00;
    params[1]= 0x1B;//frame rate = 70
    bsp_lcd_write_data(lcd_config, params, 2);

    bsp_lcd_write_cmd(lcd_config, ILI9341_DFC);    // Display Function Control
    params[0]= 0x0A;
    params[1]= 0xA2;
    bsp_lcd_write_data(lcd_config, params, 2);

    bsp_lcd_write_cmd(lcd_config, ILI9341_3GAMMA_EN);    // 3Gamma Function Disable
    params[0]= 0x02;
    bsp_lcd_write_data(lcd_config, params, 1);

    bsp_lcd_write_cmd(lcd_config, ILI9341_GAMMA);
    params[0]= 0x01;
    bsp_lcd_write_data(lcd_config, params, 1);

    bsp_lcd_write_cmd(lcd_config, ILI9341_PGAMMA);    //Set Gamma
    params[0]= 0x0F;
    params[1]= 0x1D;
    params[2]= 0x1A;
    params[3]= 0x0A;
    params[4]= 0x0D;
    params[5]= 0x07;
    params[6]= 0x49;
    params[7]= 0X66;
    params[8]= 0x3B;
    params[9]= 0x07;
    params[10]= 0x11;
    params[11]= 0x01;
    params[12]= 0x09;
    params[13]= 0x05;
    params[14]= 0x04;
    bsp_lcd_write_data(lcd_config, params, 15);

    bsp_lcd_write_cmd(lcd_config, ILI9341_NGAMMA);
    params[0]= 0x00;
    params[1]= 0x18;
    params[2]= 0x1D;
    params[3]= 0x02;
    params[4]= 0x0F;
    params[5]= 0x04;
    params[6]= 0x36;
    params[7]= 0x13;
    params[8]= 0x4C;
    params[9]= 0x07;
    params[10]= 0x13;
    params[11]= 0x0F;
    params[12]= 0x2E;
    params[13]= 0x2F;
    params[14]= 0x05;
    bsp_lcd_write_data(lcd_config, params, 15);

    bsp_lcd_write_cmd(lcd_config, ILI9341_RASET); //page address set
    params[0]= 0x00;
    params[1]= 0x00;
    params[2]= 0x00;
    params[3]= 0xf0; //240 rows = 0xf0
    bsp_lcd_write_data(lcd_config, params, 4);

    bsp_lcd_write_cmd(lcd_config, ILI9341_CASET);
    params[0]= 0x00;
    params[1]= 0x00;
    params[2]= 0x01;
    params[3]= 0x40; //320 columns = 0x140
    bsp_lcd_write_data(lcd_config, params, 4);

    bsp_lcd_write_cmd(lcd_config, ILI9341_RGB_INTERFACE);
    params[0] = 0xC2; //Data is fetched during falling edge of DOTCLK
    bsp_lcd_write_data(lcd_config, params, 1);

    bsp_lcd_write_cmd(lcd_config, ILI9341_INTERFACE);
    params[0] = 0x00;
    params[1] = 0x00;
    params[2] = 0x06;
    bsp_lcd_write_data(lcd_config, params, 3);

    bsp_lcd_write_cmd(lcd_config, ILI9341_SLEEP_OUT); //Exit Sleep
    delay_50ms();
    delay_50ms();
    bsp_lcd_write_cmd(lcd_config, ILI9341_DISPLAY_ON); //display on
}

void bsp_lcd_write_cmd(bsp_lcd_config_t* lcd_config, uint8_t cmd)
{
    hal_gpio_set_low(lcd_config->gpio_port_csx, lcd_config->gpio_num_csx);
    hal_gpio_set_low(lcd_config->gpio_port_dcx, lcd_config->gpio_num_dcx);
    while(!READ_BIT((lcd_config->spi)->SR, SPI_SR_TXE));
    WRITE_REG((lcd_config->spi)->DR, cmd);
    while(!READ_BIT((lcd_config->spi)->SR, SPI_SR_TXE));
    while(READ_BIT((lcd_config->spi)->SR, SPI_SR_BSY));
    hal_gpio_set_high(lcd_config->gpio_port_dcx, lcd_config->gpio_num_dcx);
    hal_gpio_set_high(lcd_config->gpio_port_csx, lcd_config->gpio_num_csx);
}

void bsp_lcd_write_data(bsp_lcd_config_t* lcd_config, uint8_t* buffer, uint32_t len)
{
    for(uint32_t i = 0; i < len; i++)
    {
        hal_gpio_set_low(lcd_config->gpio_port_csx, lcd_config->gpio_num_csx);
        while(!READ_BIT((lcd_config->spi)->SR, SPI_SR_TXE));
        WRITE_REG((lcd_config->spi)->DR, buffer[i]);
        while(!READ_BIT((lcd_config->spi)->SR, SPI_SR_TXE));
        while(READ_BIT((lcd_config->spi)->SR, SPI_SR_BSY));
        hal_gpio_set_high(lcd_config->gpio_port_csx, lcd_config->gpio_num_csx);
    }
}

void delay_50ms(void)
{
    for(uint32_t i = 0; i < 0xFFFFU * 10U; i++);    /* 50ms of delay */
}