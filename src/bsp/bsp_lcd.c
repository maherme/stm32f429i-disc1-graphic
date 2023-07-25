/************************************************************************************************//**
* @file bsp_lcd.c
*
* @brief File containing the board support package for LCD.
**/

#include "bsp_lcd.h"
#include "stm32f429xx.h"
#include "stm32f4xx.h"
#include "ili9341_reg.h"

#define LCD_SCL_PIN     GPIO_PIN_7
#define LCD_SCL_PORT    GPIOF
#define LCD_SDA_PIN     GPIO_PIN_9
#define LCD_SDA_PORT    GPIOF
#define LCD_RESX_PIN    GPIO_PIN_7
#define LCD_RESX_PORT   GPIOA
#define LCD_CSX_PIN     GPIO_PIN_2
#define LCD_CSX_PORT    GPIOC
#define LCD_DCX_PIN     GPIO_PIN_13
#define LCD_DCX_PORT    GPIOD

#define BSP_LCD_RESX_HIGH()     SET_BIT(GPIOA->ODR, GPIO_ODR_OD7)
#define BSP_LCD_RESX_LOW()      CLEAR_BIT(GPIOA->ODR, GPIO_ODR_OD7)
#define BSP_LCD_CSX_HIGH()      SET_BIT(GPIOC->ODR, GPIO_ODR_OD2)
#define BSP_LCD_CSX_LOW()       CLEAR_BIT(GPIOC->ODR, GPIO_ODR_OD2)
#define BSP_LCD_DCX_HIGH()      SET_BIT(GPIOD->ODR, GPIO_ODR_OD13)
#define BSP_LCD_DCX_LOW()       CLEAR_BIT(GPIOD->ODR, GPIO_ODR_OD13)

/***************************************************************************************************/
/*                                       Static Function Prototypes                                */
/***************************************************************************************************/

/**
 * @brief .
 * @return void.
 */
static void bsp_lcd_pin_init(void);

/**
 * @brief .
 * @return void.
 */
static void bsp_lcd_spi_enable(SPI_TypeDef* pSPI);

/**
 * @brief .
 * @return void.
 */
static void bsp_lcd_spi_init(SPI_TypeDef* pSPI);

/**
 * @brief .
 * @return void.
 */
static void bsp_lcd_reset(void);

/**
 * @brief .
 * @return void.
 */
static void bsp_lcd_config(SPI_TypeDef* pSPI);

void bsp_lcd_write_cmd(SPI_TypeDef* pSPI, uint8_t cmd);
void bsp_lcd_write_data(SPI_TypeDef* pSPI, uint8_t* buffer, uint32_t len);

/***************************************************************************************************/
/*                                       Public API Definitions                                    */
/***************************************************************************************************/

void bsp_lcd_init(void)
{
    bsp_lcd_pin_init();
    bsp_lcd_spi_init(SPI5);
    bsp_lcd_spi_enable(SPI5);
    bsp_lcd_reset();
    bsp_lcd_config(SPI5);
}

/***************************************************************************************************/
/*                                       Static Function Definitions                               */
/***************************************************************************************************/

static void bsp_lcd_pin_init(void)
{
    /* Enable clock for peripherals */
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN);
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIODEN);
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOFEN);

    /* Configure RESX */
    /* General purpose output mode */
    MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER7, _VAL2FLD(GPIO_MODER_MODER7, 1));
    /* Push-pull */
    MODIFY_REG(GPIOA->OTYPER, GPIO_OTYPER_OT7, _VAL2FLD(GPIO_OTYPER_OT7, 0));
    /* High speed */
    MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEED7, _VAL2FLD(GPIO_OSPEEDR_OSPEED7, 2));

    /* Configure CSX */
    /* General purpose output mode */
    MODIFY_REG(GPIOC->MODER, GPIO_MODER_MODER2, _VAL2FLD(GPIO_MODER_MODER2, 1));
    /* Push-pull */
    MODIFY_REG(GPIOC->OTYPER, GPIO_OTYPER_OT2, _VAL2FLD(GPIO_OTYPER_OT2, 0));
    /* High speed */
    MODIFY_REG(GPIOC->OSPEEDR, GPIO_OSPEEDR_OSPEED2, _VAL2FLD(GPIO_OSPEEDR_OSPEED2, 2));

    /* Configure DCX */
    /* General purpose output mode */
    MODIFY_REG(GPIOD->MODER, GPIO_MODER_MODER13, _VAL2FLD(GPIO_MODER_MODER13, 1));
    /* Push-pull */
    MODIFY_REG(GPIOD->OTYPER, GPIO_OTYPER_OT13, _VAL2FLD(GPIO_OTYPER_OT13, 0));
    /* High speed */
    MODIFY_REG(GPIOD->OSPEEDR, GPIO_OSPEEDR_OSPEED13, _VAL2FLD(GPIO_OSPEEDR_OSPEED13, 2));

    /* Configure SPI CLK */
    /* General purpose output mode */
    MODIFY_REG(GPIOF->MODER, GPIO_MODER_MODER7, _VAL2FLD(GPIO_MODER_MODER7, 2));
    /* Push-pull */
    MODIFY_REG(GPIOF->OTYPER, GPIO_OTYPER_OT7, _VAL2FLD(GPIO_OTYPER_OT7, 0));
    /* High speed */
    MODIFY_REG(GPIOF->OSPEEDR, GPIO_OSPEEDR_OSPEED7, _VAL2FLD(GPIO_OSPEEDR_OSPEED7, 2));
    /* Alternate functionality 5 */
    MODIFY_REG(GPIOF->AFR[0], GPIO_AFRL_AFSEL7, _VAL2FLD(GPIO_AFRL_AFSEL7, 5));

    /* Configure SPI SDA */
    /* General purpose output mode */
    MODIFY_REG(GPIOF->MODER, GPIO_MODER_MODER9, _VAL2FLD(GPIO_MODER_MODER9, 2));
    /* Push-pull */
    MODIFY_REG(GPIOF->OTYPER, GPIO_OTYPER_OT9, _VAL2FLD(GPIO_OTYPER_OT9, 0));
    /* High speed */
    MODIFY_REG(GPIOF->OSPEEDR, GPIO_OSPEEDR_OSPEED9, _VAL2FLD(GPIO_OSPEEDR_OSPEED9, 2));
    /* Alternate functionality 5 */
    MODIFY_REG(GPIOF->AFR[1], GPIO_AFRH_AFSEL9, _VAL2FLD(GPIO_AFRH_AFSEL9, 5));

    /* Set CSX pin to high */
    SET_BIT(GPIOC->ODR, GPIO_ODR_OD2);
    /* Set RESX pin to high */
    SET_BIT(GPIOA->ODR, GPIO_ODR_OD7);
    /* Set DCX pin to high */
    SET_BIT(GPIOD->ODR, GPIO_ODR_OD13);
}

static void bsp_lcd_spi_enable(SPI_TypeDef* pSPI)
{
    SET_BIT(pSPI->CR1, SPI_CR1_SPE);
}

static void bsp_lcd_spi_init(SPI_TypeDef* pSPI)
{
    /* Enable clock for peripheral */
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SPI5EN);

    /* Controller mode */
    SET_BIT(pSPI->CR1, SPI_CR1_MSTR);
    /* BIDI mode enable */
    SET_BIT(pSPI->CR1, SPI_CR1_BIDIMODE);
    /* BIDIOE enable */
    SET_BIT(pSPI->CR1, SPI_CR1_BIDIOE);
    /* DFF set to 8 bits */
    CLEAR_BIT(pSPI->CR1, SPI_CR1_DFF);
    /* SSM enable */
    SET_BIT(pSPI->CR1, SPI_CR1_SSM);
    /* SSI enable */
    SET_BIT(pSPI->CR1, SPI_CR1_SSI);
    /* Send MSB first */
    CLEAR_BIT(pSPI->CR1, SPI_CR1_LSBFIRST);
    /* SPI clock set to 90MHz/16 = 5.625MHz */
    MODIFY_REG(pSPI->CR1, SPI_CR1_BR, _VAL2FLD(SPI_CR1_BR, 3));
    /* CPOL set to 0 */
    CLEAR_BIT(pSPI->CR1, SPI_CR1_CPOL);
    /* CPHA set to 0 */
    CLEAR_BIT(pSPI->CR1, SPI_CR1_CPHA);
    /* SPI Motorola frame format */
    CLEAR_BIT(pSPI->CR2, SPI_CR2_FRF);
}

static void bsp_lcd_reset(void)
{
    BSP_LCD_RESX_LOW();
    for(uint32_t i = 0; i < 0xFFFFU * 20U; i++);    /* 50ms of delay */
    BSP_LCD_RESX_HIGH();
    for(uint32_t i = 0; i < 0xFFFFU * 20U; i++);
}

static void bsp_lcd_config(SPI_TypeDef* pSPI)
{
    uint8_t params[15] = {0};

    bsp_lcd_write_cmd(pSPI, ILI9341_SWRESET);
    bsp_lcd_write_cmd(pSPI, ILI9341_POWERB);
    params[0] = 0x00;
    params[1] = 0xD9;
    params[2] = 0x30;
    bsp_lcd_write_data(pSPI, params, 3);
}

void bsp_lcd_write_cmd(SPI_TypeDef* pSPI, uint8_t cmd)
{
    BSP_LCD_CSX_LOW();
    BSP_LCD_DCX_LOW();
    while(!READ_BIT(pSPI->SR, SPI_SR_TXE));
    WRITE_REG(pSPI->DR, cmd);
    while(!READ_BIT(pSPI->SR, SPI_SR_TXE));
    while(READ_BIT(pSPI->SR, SPI_SR_BSY));
    BSP_LCD_DCX_HIGH();
    BSP_LCD_CSX_HIGH();
}

void bsp_lcd_write_data(SPI_TypeDef* pSPI, uint8_t* buffer, uint32_t len)
{
    for(uint32_t i = 0; i < len; i++)
    {
        BSP_LCD_CSX_LOW();
        while(!READ_BIT(pSPI->SR, SPI_SR_TXE));
        WRITE_REG(pSPI->DR, buffer[i]);
        while(!READ_BIT(pSPI->SR, SPI_SR_TXE));
        while(READ_BIT(pSPI->SR, SPI_SR_BSY));
        BSP_LCD_CSX_HIGH();
    }
}