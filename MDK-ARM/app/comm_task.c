#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "main.h"
#include "communication.h"
#include "data_fifo.h"
#include "uart_dma.h"
#include "string.h" // memset
#include "comm_task.h"
#include "jr_comm.h" // communication data structure declarations
#include "info_interactive.h"

UBaseType_t freq_info_surplus;
UBaseType_t pc_comm_surplus;

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
//  pc_rx_obj.buff[0] = pc_dma_rxbuff[0];
//  pc_rx_obj.buff[1] = pc_dma_rxbuff[1];
  
  /* initial pc data unpack object */
  pc_unpack_obj.data_fifo = &pc_rxdata_fifo;
  pc_unpack_obj.p_header = (frame_header_t *)pc_unpack_obj.protocol_packet;
  pc_unpack_obj.index = 0;
  pc_unpack_obj.data_len = 0;
  pc_unpack_obj.unpack_step = STEP_HEADER_SOF;
  
  memset(&glb_chassis_states, 0, sizeof(chassis_states_t));
//
//  pc_send_mesg.version_info_data.num[0] = 2;
//  pc_send_mesg.version_info_data.num[1] = 0;
//  pc_send_mesg.version_info_data.num[2] = 1;
//  pc_send_mesg.version_info_data.num[3] = 8;
	
}

/* USER CODE BEGIN Header_comm_task */
/**
* @brief Function implementing the taskComm thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_comm_task */
//void comm_task(void const *argu)
//{
//  /* USER CODE BEGIN comm_task */
//  /* Infinite loop */
//  for(;;)
//  {
//		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
//		HAL_UART_Transmit(&huart2, (uint8_t *)"hello\r\n", 7, 100);
//
//		while (1) {
//
//
//		}
//
//    osDelay(1000);
//  }
//  /* USER CODE END comm_task */
//}

void data_packet_pack(uint16_t cmd_id, uint8_t *p_data, uint16_t len, uint8_t sof)
{
  uint8_t tx_buf[PROTOCAL_FRAME_MAX_SIZE];

  uint16_t frame_length = HEADER_LEN + CMD_LEN + len + CRC_LEN;

  protocol_packet_pack(cmd_id, p_data, len, sof, tx_buf);

  /* use mutex operation */
  if (sof == UP_REG_ID)
    fifo_s_puts(&pc_txdata_fifo, tx_buf, frame_length);
  else
    return ;
}

void get_upload_data(void)
{
  taskENTER_CRITICAL();
  //taskENTER_CRITICAL_FROM_ISR()

  get_infantry_info();
//  get_custom_data_info();

  taskEXIT_CRITICAL();
  //taskEXIT_CRITICAL_FROM_ISR
}


uint32_t comm_time_last;
int comm_time_ms;
int count_cali_count = 0;
extern TaskHandle_t pc_unpack_task_t;

void freq_info_task(void const *argu) {
  uint32_t comm_wake_time = osKernelSysTick();
  while (1)
  {
    comm_time_ms = HAL_GetTick() - comm_time_last;
    comm_time_last = HAL_GetTick();

    get_upload_data();

    if (count_cali_count == 1)   //50Hz
    {
      count_cali_count = 0;
      data_packet_pack(CHASSIS_DATA_ID, (uint8_t *)&pc_send_mesg.chassis_information,
                       sizeof(chassis_info_t), UP_REG_ID);
    } else {                      //50Hz
      count_cali_count ++;
    }
    osSignalSet(pc_unpack_task_t, PC_UART_TX_SIGNAL);

    freq_info_surplus = uxTaskGetStackHighWaterMark(NULL);

    osDelayUntil(&comm_wake_time, COMM_TASK_PERIOD);  //100Hz

  }
}

extern TaskHandle_t freq_info_task_t;
void comm_task(void const *argu) {

  osEvent event;

  taskENTER_CRITICAL();
  /* open pc uart receive it */
  computer_uart_init();
  /* create periodic information task */
  osThreadDef(commTask, freq_info_task, osPriorityNormal, 0, 256);
  freq_info_task_t = osThreadCreate(osThread(commTask), NULL);
  taskEXIT_CRITICAL();

  /* USER CODE BEGIN comm_task */
  /* Infinite loop */
  while (1) {
    event = osSignalWait(PC_UART_TX_SIGNAL | \
                         PC_UART_IDLE_SIGNAL, osWaitForever);
    /* Haowen: upon IDLE, will receive data from PC */
    if (event.status == osEventSignal) {
      //receive pc data puts fifo
      if (event.value.signals & PC_UART_IDLE_SIGNAL) {
        dma_buffer_to_unpack_buffer(&pc_rx_obj, UART_IDLE_IT);
        unpack_fifo_data(&pc_unpack_obj, UP_REG_ID);
      }

      //send data to pc system
      if (event.value.signals & PC_UART_TX_SIGNAL) {
        send_packed_fifo_data(&pc_txdata_fifo, UP_REG_ID);
      }

    }

    pc_comm_surplus = uxTaskGetStackHighWaterMark(NULL);
  }

}
  /* USER CODE END comm_task */

