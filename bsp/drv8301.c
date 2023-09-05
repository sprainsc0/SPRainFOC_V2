#include "drv8301.h"
#include <stdint.h>
#include "delay.h"

static void drv_en(bool en)
{
	if (en)
		HAL_GPIO_WritePin(SPI1_CS2_GPIO_Port, SPI1_CS2_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(SPI1_CS2_GPIO_Port, SPI1_CS2_Pin, GPIO_PIN_SET);
}

static void drv8301_write_reg(int reg, int data) 
{
	uint16_t rx;
	uint16_t out = 0;
	out |= (reg & 0x0F) << 11;
	out |= data & 0x7FF;

	drv_en(true);
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&out, (uint8_t *)&rx, 1, 1);
	drv_en(false);
}

void hal_drv8301_init(void)
{
	drv8301_write_reg(2, 0x0430);
	drv8301_write_reg(2, 0x0430);
}