#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "main.h"
#include "communication.h"
#include "data_fifo.h"
#include "uart_dma.h"
#include "string.h" // memset
#include "comm_task.h"
#include "jr_comm.h" // communication data structure declarations

/* pc receive data fifo and buffer */
static osMutexId pc_rxdata_mutex;
fifo_s_t  pc_rxdata_fifo;
uint8_t   pc_rxdata_buf[COMPUTER_FIFO_BUFLEN];
/* pc send data fifo and buffer */
static osMutexId pc_txdata_mutex;
static fifo_s_t  pc_txdata_fifo;
static uint8_t   pc_txdata_buf[COMPUTER_FIFO_BUFLEN];
/* pc system dma receive data object */
static uart_dma_rxdata_t pc_rx_obj;

/* unpack object */
static unpack_data_t pc_unpack_obj;

chassis_states_t glb_chassis_states;

TaskHandle_t uplinkTaskHandle;

void communicate_param_init(void)
{
  
  /* create the pc_rxdata_mutex mutex  */
  osMutexDef(pc_rxdata_mutex);
  pc_rxdata_mutex = osMutexCreate(osMutex(pc_rxdata_mutex));
  
  /* create the pc_txdata_mutex mutex  */
  osMutexDef(pc_txdata_mutex);
  pc_txdata_mutex = osMutexCreate(osMutex(pc_txdata_mutex));

  /* pc data fifo init */
  fifo_s_init(&pc_rxdata_fifo, pc_rxdata_buf, COMPUTER_FIFO_BUFLEN, pc_rxdata_mutex);
  fifo_s_init(&pc_txdata_fifo, pc_txdata_buf, COMPUTER_FIFO_BUFLEN, pc_txdata_mutex);
  
  /* initial pc data dma receiver object */
  pc_rx_obj.huart = &huart2;
  pc_rx_obj.data_fifo = &pc_rxdata_fifo;
  pc_rx_obj.buff_size = UART_RX_DMA_SIZE;
  pc_rx_obj.buff[0] = pc_dma_rxbuff[0];
  pc_rx_obj.buff[1] = pc_dma_rxbuff[1];
  
  /* initial pc data unpack object */
  pc_unpack_obj.data_fifo = &pc_rxdata_fifo;
  pc_unpack_obj.p_header = (frame_header_t *)pc_unpack_obj.protocol_packet;
  pc_unpack_obj.index = 0;
  pc_unpack_obj.data_len = 0;
  pc_unpack_obj.unpack_step = STEP_HEADER_SOF;
  
  memset(&glb_chassis_states, 0, sizeof(chassis_states_t));
}

void uplink_task(void const *argu)
{
	while (1)
	{
		/* TODO: do uplink stuff */
		HAL_UART_Transmit(&huart2, (uint8_t *)"hi\r\n", 4, 100);
		HAL_Delay(1000);
	}
}

/* USER CODE BEGIN Header_comm_task */
/**
* @brief Function implementing the taskComm thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_comm_task */
void comm_task(void const *argu)
{
  /* USER CODE BEGIN comm_task */
  osEvent event;
	
	taskENTER_CRITICAL();
	/* Setup uplink thread */
	computer_uart_init();
	
	osThreadDef(taskUplink, uplink_task, osPriorityNormal, 0, 256);
	uplinkTaskHandle = osThreadCreate(osThread(taskUplink), NULL);
	taskEXIT_CRITICAL();
	
	//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	
  while (1)
  {
		event = osSignalWait(PC_UART_TX_SIGNAL | \
		                     PC_UART_IDLE_SIGNAL, osWaitForever);
		
		if (event.status == osEventSignal) {
			// receive pc data puts fifo
			if (event.value.signals & PC_UART_IDLE_SIGNAL) {
				dma_buffer_to_unpack_buffer(&pc_rx_obj, UART_IDLE_IT);
				unpack_fifo_data(&pc_unpack_obj, UP_REG_ID);
				
				// blink LED2 as life indicator
				HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
			}
			
			if (event.value.signals & PC_UART_TX_SIGNAL) {
				send_packed_fifo_data(&pc_txdata_fifo, UP_REG_ID);
			}
		}
		
		//HAL_UART_Transmit(&huart2, (uint8_t *)"hello\r\n", 7, 100);
    //osDelay(1000);
  }
  /* USER CODE END comm_task */
}
