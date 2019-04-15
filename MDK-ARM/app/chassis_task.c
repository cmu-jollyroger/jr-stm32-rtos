#include "chassis_task.h"
#include "jr_comm.h"

/* chassis task global parameter */
chassis_t chassis;

uint32_t chassis_time_last;
int chassis_time_ms;
void chassis_task(void const *argu)
{
  chassis_time_ms = HAL_GetTick() - chassis_time_last;
  chassis_time_last = HAL_GetTick();

  get_chassis_info();
//    get_chassis_mode();

  switch (chassis.ctrl_mode)
  {
    case CHASSIS_MOVING:
    {
      chassis.vx = pc_recv_mesg.chassis_control_data.x_spd;
      chassis.vy = pc_recv_mesg.chassis_control_data.y_spd;
      chassis.vw = pc_recv_mesg.chassis_control_data.w_info.w_spd;

//      int16_t l_speed = pc_recv_mesg.chassis_control_data.x_spd;
//      int16_t r_speed = pc_recv_mesg.chassis_control_data.y_spd;
//
//      // L: 1, 3, -
//      // R: 0, 2, +
//      chassis.wheel_spd_ref[0] = (r_speed);
//      chassis.wheel_spd_ref[1] = - (l_speed);
//      chassis.wheel_spd_ref[2] = (r_speed);
//      chassis.wheel_spd_ref[3] = - (l_speed);
    }break;

    case CHASSIS_STOP:
    {
      chassis_stop_handler();
    }break;

    default:
    {
      chassis_stop_handler();
    }break;
  }

  for (int i = 0; i < 4; i++)
  {
    chassis.current[i] = pid_calc(
            &pid_spd[i],
            chassis.wheel_spd_fdb[i],
            chassis.wheel_spd_ref[i]
    );
    //chassis.current[i] = chassis.wheel_spd_ref[i]; // disable PID for testing
  }

  memcpy(glb_cur.chassis_cur, chassis.current, sizeof(chassis.current));
  osSignalSet(can_msg_send_task_t, CHASSIS_MOTOR_MSG_SEND);

  chassis_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
}
