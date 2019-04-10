#include "cmsis_os.h"
#include "string.h"
#include "protocol.h"
#include "jr_comm.h"

// send and receive
send_pc_t    pc_send_mesg;
receive_pc_t pc_recv_mesg;

// for debug
int pc_seq            = 0;
int once_lost_num     = 0;
int lost_pack_percent = 0;

int pack_num_cnt   = 0;
int lost_num_sum_t = 0;
int pack_lost      = 0;

void pc_data_handler(uint8_t *p_frame)
{
  frame_header_t *p_header = (frame_header_t*)p_frame;
  memcpy(p_header, p_frame, HEADER_LEN);

  uint16_t data_length = p_header->data_length;
  uint16_t cmd_id      = *(uint16_t *)(p_frame + HEADER_LEN);
  uint8_t *data_addr   = p_frame + HEADER_LEN + CMD_LEN;

  //lost pack monitor
  pack_num_cnt++;
  
  if (pack_num_cnt <= 100)
  {
    once_lost_num = p_header->seq - pc_seq - 1;
    
    if (once_lost_num < 0)
    {
      once_lost_num += 256;
    }
    
    lost_num_sum_t += once_lost_num;
  }
  else
  {
    lost_pack_percent = lost_num_sum_t;
    lost_num_sum_t    = 0;
    pack_num_cnt      = 0;
  }
  
  
  if (once_lost_num != 0)
  {
    pack_lost = 1;
  }
  else
  {
    pack_lost = 0;
  }
  
  pc_seq = p_header->seq;
  //end lost pack monitor
  
  
  taskENTER_CRITICAL();
  
  switch (cmd_id)
  {
    case CHASSIS_CTRL_ID:
    {
			// command to chassis
      memcpy(&pc_recv_mesg.chassis_control_data, data_addr, data_length);
      chassis.ctrl_mode = (chassis_mode_e)pc_recv_mesg.chassis_control_data.ctrl_mode;
      
    }
    break;
    
    case ERROR_LEVEL_ID:
    {
      memcpy(&pc_recv_mesg.global_error_level, data_addr, data_length);
      
      pc_state = pc_recv_mesg.global_error_level.err_level;

    }
    break;
    
    case INFANTRY_STRUCT_ID:
      memcpy(&pc_recv_mesg.structure_data, data_addr, data_length);
    break;
		
  }
  
  taskEXIT_CRITICAL();
}