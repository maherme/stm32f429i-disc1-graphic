#include "stm32f4xx.h"

typedef struct
{
    SPI_TypeDef* spi;
}hal_spi_config_t;

void hal_spi_enable_clock(SPI_TypeDef* spi);
void hal_spi_enable(SPI_TypeDef* spi);
void hal_spi_init(hal_spi_config_t* spi_config);