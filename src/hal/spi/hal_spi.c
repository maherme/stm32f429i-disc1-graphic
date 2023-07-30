#include <stdint.h>
#include "hal_spi.h"
#include "stm32f4xx.h"

void hal_spi_enable_clock(SPI_TypeDef* spi)
{
    switch((intptr_t)spi)
    {
        case (intptr_t)SPI1:
            SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SPI1EN);
            break;
        case (intptr_t)SPI2:
            SET_BIT(RCC->APB1ENR, RCC_APB1ENR_SPI2EN);
            break;
        case (intptr_t)SPI3:
            SET_BIT(RCC->APB1ENR, RCC_APB1ENR_SPI3EN);
            break;
        case (intptr_t)SPI4:
            SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SPI4EN);
            break;
        case (intptr_t)SPI5:
            SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SPI5EN);
            break;
        case (intptr_t)SPI6:
            SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SPI6EN);
            break;
        default:
            break;
    }
}

void hal_spi_enable(SPI_TypeDef* spi)
{
    SET_BIT(spi->CR1, SPI_CR1_SPE);
}

void hal_spi_init(hal_spi_config_t* spi_config)
{
    /* Enable peripheral clock */
    hal_spi_enable_clock(spi_config->spi);

    /* Controller mode */
    SET_BIT((spi_config->spi)->CR1, SPI_CR1_MSTR);
    /* BIDI mode enable */
    SET_BIT((spi_config->spi)->CR1, SPI_CR1_BIDIMODE);
    /* BIDIOE enable */
    SET_BIT((spi_config->spi)->CR1, SPI_CR1_BIDIOE);
    /* DFF set to 8 bits */
    CLEAR_BIT((spi_config->spi)->CR1, SPI_CR1_DFF);
    /* SSM enable */
    SET_BIT((spi_config->spi)->CR1, SPI_CR1_SSM);
    /* SSI enable */
    SET_BIT((spi_config->spi)->CR1, SPI_CR1_SSI);
    /* Send MSB first */
    CLEAR_BIT((spi_config->spi)->CR1, SPI_CR1_LSBFIRST);
    /* SPI clock set to 90MHz/16 = 5.625MHz */
    MODIFY_REG((spi_config->spi)->CR1, SPI_CR1_BR, _VAL2FLD(SPI_CR1_BR, 3));
    /* CPOL set to 0 */
    CLEAR_BIT((spi_config->spi)->CR1, SPI_CR1_CPOL);
    /* CPHA set to 0 */
    CLEAR_BIT((spi_config->spi)->CR1, SPI_CR1_CPHA);
    /* SPI Motorola frame format */
    CLEAR_BIT((spi_config->spi)->CR2, SPI_CR2_FRF);
}