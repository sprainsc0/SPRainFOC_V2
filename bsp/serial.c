#include "serial.h"
#include "ringbuffer.h"
#include <semphr.h>

static struct ringbuffer uart_ring;
static SemaphoreHandle_t uart_sem;
#define UARTRINGBUFFER_SIZE     1024
static uint8_t uart_buffer[UARTRINGBUFFER_SIZE];

#define UARTDMABUFFERSIZE       256
static uint8_t uart_dma[UARTDMABUFFERSIZE];

UART_HandleTypeDef *uart_handle = &huart3;

static void HAL_UART3_TxCpltCallback(UART_HandleTypeDef *huart)
{
	static portBASE_TYPE xHigherPriorityTaskWoken;
	if(uart_sem != NULL) {
		xSemaphoreGiveFromISR(uart_sem, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
	
}

static void HAL_UART3_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_DMAStop(huart);
	uint16_t lenght = UARTDMABUFFERSIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);
	ringbuffer_put(&uart_ring, uart_dma, lenght);
	HAL_UART_Receive_DMA(huart, uart_dma, UARTDMABUFFERSIZE);
}

void hal_uart_irq(void)
{
	if(__HAL_UART_GET_FLAG(uart_handle, UART_FLAG_IDLE) != RESET) {
        __HAL_UART_CLEAR_IDLEFLAG(uart_handle);
        
        HAL_UART_DMAStop(uart_handle);
        __HAL_UART_FLUSH_DRREGISTER(uart_handle);
        
        uint16_t lenght = UARTDMABUFFERSIZE - __HAL_DMA_GET_COUNTER(uart_handle->hdmarx);
        // put data to ringbuffer
        ringbuffer_put(&uart_ring, uart_dma, lenght);
    
        HAL_UART_Receive_DMA(uart_handle, uart_dma, UARTDMABUFFERSIZE);
    }
}

void hal_uart_init(void)
{
	ringbuffer_init(&uart_ring, uart_buffer, UARTRINGBUFFER_SIZE);

	uart_sem = xSemaphoreCreateBinary();

	HAL_UART_RegisterCallback(uart_handle, HAL_UART_TX_COMPLETE_CB_ID, HAL_UART3_TxCpltCallback);
	HAL_UART_RegisterCallback(uart_handle, HAL_UART_RX_COMPLETE_CB_ID, HAL_UART3_RxCpltCallback);

	__HAL_UART_ENABLE_IT(uart_handle, UART_IT_IDLE);
    //__HAL_UART_ONE_BIT_SAMPLE_ENABLE(&huart3);

	HAL_UART_Receive_DMA(uart_handle, uart_dma, UARTDMABUFFERSIZE);
}

uint32_t hal_uart_write(void *data, uint32_t size)
{
	while(uart_handle->gState != HAL_UART_STATE_READY);
    
    if(HAL_UART_Transmit_DMA(uart_handle, (uint8_t *)data, size) != HAL_OK) {
        return 0;
    }
    xSemaphoreTake(uart_sem, 1000);

	return size;
}

uint32_t hal_uart_read(void *data, uint32_t size)
{
	return ringbuffer_get(&uart_ring, (uint8_t *)data, size);
}

uint32_t hal_uart_valid(void)
{
	return ringbuffer_data_len(&uart_ring);
}
