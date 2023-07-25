/************************************************************************************************//**
* @file systeminit.c
*
* @brief File containing the APIs for initiating the device.
*
* Public Functions:
*       - void SystemInit(void)
*
* @note
*       For further information about functions refer to the corresponding header file.
**/

#include <stdint.h>
#include "system_stm32f4xx.h"
#include "stm32f4xx.h"

/** @brief Declared variable in system_stm32f4xxx.h for storing the system core clock */
uint32_t SystemCoreClock = 180000000;

/***********************************************************************************************************/
/*                                       Static Function Prototypes                                        */
/***********************************************************************************************************/

/**
 * @brief Function for configuring the system clock.
 * @return void
 */
static inline __attribute__((always_inline)) void configure_clock(void);

/**
 * @brief Function for configuring the mco1.
 * @return void
 */
__attribute__((unused)) static inline __attribute__((always_inline)) void configure_mco1(void);

/***************************************************************************************************/
/*                                       Public API Definitions                                    */
/***************************************************************************************************/

/**
 * @brief Function for initializing the device (CMSIS).
 * @return void
 */
void SystemInit(void)
{
//    configure_mco1();
    configure_clock();
}

/***************************************************************************************************/
/*                                       Static Function Definitions                               */
/***************************************************************************************************/

static inline __attribute__((always_inline)) void configure_clock(void)
{
    /* Configure FLASH latency */
    MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, _VAL2FLD(FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_5WS));

    /* Configure Over Drive */
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN);
    /* Set VOS to 0b11 */
    MODIFY_REG(PWR->CR, PWR_CR_VOS, 3);
    /* Activate over drive mode */
    SET_BIT(PWR->CR, PWR_CR_ODEN);
    /* Wait for over drive ready */
    while(!READ_BIT(PWR->CSR, PWR_CSR_ODRDY));
    /* Over drive switch enable */
    SET_BIT(PWR->CR, PWR_CR_ODSWEN);

    /* Enable HSE */
    SET_BIT(RCC->CR, RCC_CR_HSEON);

    /* Wait until HSE is stable */
    while(!READ_BIT(RCC->CR, RCC_CR_HSERDY));

    /* Configure PLL */
    MODIFY_REG(
        RCC->PLLCFGR,
        RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN | RCC_PLLCFGR_PLLP,
        _VAL2FLD(RCC_PLLCFGR_PLLM, 8) | _VAL2FLD(RCC_PLLCFGR_PLLN, 180) |
        _VAL2FLD(RCC_PLLCFGR_PLLP, 0)
    );

    /* Enable PLL */
    SET_BIT(RCC->CR, RCC_CR_PLLON);

    /* Wait until PLL is stable */
    while(!READ_BIT(RCC->CR, RCC_CR_PLLRDY));

    /* Switch system clock to PLL */
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, _VAL2FLD(RCC_CFGR_SW, RCC_CFGR_SW_PLL));

    /* Configure PPRE1 */
    MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, _VAL2FLD(RCC_CFGR_HPRE, 0));
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, _VAL2FLD(RCC_CFGR_PPRE1, 5));
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, _VAL2FLD(RCC_CFGR_PPRE2, 4));

    /* Wait until PLL is used */
    while(READ_BIT(RCC->CFGR, RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    /* Disable HSI */
    CLEAR_BIT(RCC->CR, RCC_CR_HSION);
}

static inline  __attribute__((always_inline)) void configure_mco1(void)
{
    /* Configure MCO1, source PLLCLK and MCO1PRE is 2 */
    MODIFY_REG(
        RCC->CFGR,
        RCC_CFGR_MCO1 | RCC_CFGR_MCO1PRE,
        _VAL2FLD(RCC_CFGR_MCO1, 3) | _VAL2FLD(RCC_CFGR_MCO1PRE, 4)
    );

    /* Enable GPIOA (MCO1 is connnected to PA8) */
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);

    /* Configure PA8 as medium speed and alternate function mode */
    MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDER_OSPEEDR8, _VAL2FLD(GPIO_OSPEEDR_OSPEED8, 1));
    MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER8, _VAL2FLD(GPIO_MODER_MODER8, 2));
}