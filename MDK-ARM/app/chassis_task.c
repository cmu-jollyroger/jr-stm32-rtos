#include "chassis_task.h"
#include "MeEncoderNew.h"
#include "buzzer.h"

/**
 * @file mecanum_ctrl.hpp
 * @author Haowen Shi
 * @date 16 Mar 2019
 * @brief Mecanum wheel control constants and helpers
 */

#ifndef __MEC_CTRL_HPP__
#define __MEC_CTRL_HPP__

/* math relevant */
/* radian coefficient */
#define RADIAN_COEF (57.3f)
/* circumference ratio */
#define PI (3.14159f)

/** @brief Diameter of mecanum wheels (mm) */
#define WHEEL_DIAMETER (100)
/** @brief Perimeter of mecanum wheels (mm) */
#define WHEEL_PERIMETER (PI * WHEEL_DIAMETER)
/** @brief Wheel track distance (mm) */
#define WHEEL_TRACK (500)
/** @brief Wheel base distance (mm) */
#define WHEEL_BASE (550)

#define MOT_DEG_PER_ROT_DEG (6.2f)
#define MOT_DEG_PER_DIS_MM (1.116f)
#define MOT_TRA_SPD (30)
#define MOT_ROT_SPD (50)

/** @brief The deceleration ratio of chassis motor */
#define CHASSIS_DECELE_RATIO (1.0f)
/* single 3508 motor maximum speed, unit is rpm */
#define MAX_WHEEL_RPM (200)   //44rpm = 350mm/s
/* chassis maximum translation speed, unit is mm/s */
#define MAX_CHASSIS_VX_SPEED (330)  //41.5rpm
#define MAX_CHASSIS_VY_SPEED (330)
/* chassis maximum rotation speed, unit is degree/s */
#define MAX_CHASSIS_VR_SPEED (50)

#define VAL_LIMIT(val, min, max) \
do {\
if((val) <= (min))\
{\
  (val) = (min);\
}\
else if((val) >= (max))\
{\
  (val) = (max);\
}\
} while(0)\

/* arrangement of motors */
#define MOTOR_FR (2)
#define MOTOR_FL (0)
#define MOTOR_BL (1)
#define MOTOR_BR (3)

/** @brief Mecanum calculation interface */
void mecanum_calc(
    float vx,
    float vy,
    float vw,
    int16_t speed[]
);
		
void mecanum_calc(float vx, float vy, float vw, int16_t speed[]);
		
#endif /* __MEC_CTRL_HPP__ */

/**
  * @brief mecanum chassis velocity decomposition
  * @param input : ?=+vx(mm/s)  ?=+vy(mm/s)  ccw=+vw(deg/s)
  *        output: every wheel speed(rpm)
  * @note  1=FR 2=FL 3=BL 4=BR
  */
void mecanum_calc(float vx, float vy, float vw, int16_t speed[])
{
    static float rotate_ratio_fr = 1.f;
    static float rotate_ratio_fl = 1.f;
    static float rotate_ratio_bl = 1.f;
    static float rotate_ratio_br = 1.f;
    static float wheel_rpm_ratio;
    
    float rotate_x_offset = 0;
    float rotate_y_offset = 0;
    
    rotate_ratio_fr = ((WHEEL_BASE + WHEEL_TRACK) / 2.0f \
                        - rotate_x_offset + rotate_y_offset) / RADIAN_COEF;
    rotate_ratio_fl = ((WHEEL_BASE + WHEEL_TRACK) / 2.0f \
                        - rotate_x_offset - rotate_y_offset) / RADIAN_COEF;
    rotate_ratio_bl = ((WHEEL_BASE + WHEEL_TRACK) / 2.0f \
                        + rotate_x_offset - rotate_y_offset) / RADIAN_COEF;
    rotate_ratio_br = ((WHEEL_BASE + WHEEL_TRACK) / 2.0f \
                        + rotate_x_offset + rotate_y_offset) / RADIAN_COEF;

    wheel_rpm_ratio = 60.0f / (WHEEL_PERIMETER * (float) CHASSIS_DECELE_RATIO);
    
    VAL_LIMIT(vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED);  //mm/s
    VAL_LIMIT(vy, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED);  //mm/s
    VAL_LIMIT(vw, -MAX_CHASSIS_VR_SPEED, MAX_CHASSIS_VR_SPEED);  //deg/s
    
    int16_t wheel_rpm[4];
    float   max = 0;
    
    wheel_rpm[MOTOR_FR] = (-vx - vy - vw * rotate_ratio_fr) * wheel_rpm_ratio;
    wheel_rpm[MOTOR_FL] = ( vx - vy - vw * rotate_ratio_fl) * wheel_rpm_ratio;
    wheel_rpm[MOTOR_BL] = ( vx + vy - vw * rotate_ratio_bl) * wheel_rpm_ratio;
    wheel_rpm[MOTOR_BR] = (-vx + vy - vw * rotate_ratio_br) * wheel_rpm_ratio;

    //find max item
    for (int i = 0; i < 4; i++)
    {
        if (abs(wheel_rpm[i]) > max)
        max = abs(wheel_rpm[i]);
    }
    //equal proportion
    if (max > MAX_WHEEL_RPM)
    {
        float rate = MAX_WHEEL_RPM / max;
        for (int i = 0; i < 4; i++)
        wheel_rpm[i] *= rate;
    }
    memcpy(speed, wheel_rpm, 4 * sizeof(int16_t));
}

void chassis_enc_turn_deg(int16_t degrees) {
	if (abs(degrees) < 5) return;
	move((long) (degrees * MOT_DEG_PER_ROT_DEG), MOT_ROT_SPD, 1, 0);
	move((long) (degrees * MOT_DEG_PER_ROT_DEG), MOT_ROT_SPD, 1, 1);
	move((long) (degrees * MOT_DEG_PER_ROT_DEG), MOT_ROT_SPD, 1, 2);
	move((long) (degrees * MOT_DEG_PER_ROT_DEG), MOT_ROT_SPD, 1, 3);
	while (!isTarPosReached(0)) {
		continue;
	}
}

void chassis_enc_move_mm_y(int16_t dist_mm) {
	if (abs(dist_mm) < 10) return;
	move((long) (dist_mm * MOT_DEG_PER_DIS_MM), MOT_TRA_SPD, 1, 0);
	move((long) -(dist_mm * MOT_DEG_PER_DIS_MM), MOT_TRA_SPD, 1, 1);
	move((long) (dist_mm * MOT_DEG_PER_DIS_MM), MOT_TRA_SPD, 1, 2);
	move((long) -(dist_mm * MOT_DEG_PER_DIS_MM), MOT_TRA_SPD, 1, 3);
	while (!isTarPosReached(0)) {
		continue;
	}
}

void chassis_enc_move_mm_x(int16_t dist_mm) {
	if (abs(dist_mm) < 10) return;
	move((long) -(dist_mm * MOT_DEG_PER_DIS_MM), MOT_TRA_SPD, 1, 0);
	move((long) -(dist_mm * MOT_DEG_PER_DIS_MM), MOT_TRA_SPD, 1, 1);
	move((long) (dist_mm * MOT_DEG_PER_DIS_MM), MOT_TRA_SPD, 1, 2);
	move((long) (dist_mm * MOT_DEG_PER_DIS_MM), MOT_TRA_SPD, 1, 3);
	while (!isTarPosReached(0)) {
		continue;
	}
}

/* chassis task global parameter */
chassis_t chassis;

uint32_t chassis_time_last;
int chassis_time_ms;

/**
 * @brief Get encoder feedback on chassis wheel speeds
 */
void get_motor_feedback() {
	
}

void chassis_setparam_callback(void) {
	float p = (pc_recv_mesg.structure_data.pid_vel_p);
	float i = (pc_recv_mesg.structure_data.pid_vel_i);
	float d = (pc_recv_mesg.structure_data.pid_vel_d);
	for (int ii = 0; ii < 4; ii++) {
		setSpeedPID(p, i, d, ii);
		HAL_Delay(100);
		//setSpeedPID(p, i, d, ii);
		//HAL_Delay(100);
	}
	
	p = (pc_recv_mesg.structure_data.pid_pos_p);
	i = (pc_recv_mesg.structure_data.pid_pos_i);
	d = (pc_recv_mesg.structure_data.pid_pos_d);
	for (int ii = 0; ii < 4; ii++) {
		setPosPID(p, i, d, ii);
		HAL_Delay(100);
		//setPosPID(p, i, d, ii);
		//HAL_Delay(100);
	}
	
	// TODO: sometimes this is unreliable due to I2C issues, add
	// success check here.
	
	//getSpeedPID(&p, &i, &d, 0);
	//getSpeedPID(&p, &i, &d, 1);
	//getSpeedPID(&p, &i, &d, 2);
	//getSpeedPID(&p, &i, &d, 3);
	
	buzz_n_times_with_delay(4, 50);
	
}

void chassis_task(void const *argu) {
	chassis_time_ms = HAL_GetTick() - chassis_time_last;
	chassis_time_last = HAL_GetTick();
	
	uint32_t chassis_wake_time = osKernelSysTick();
	
	chassis.enc_exec_done = 1;
	
	while (1) {
		
		chassis_ctrl_t *chassis_ctrl = (chassis_ctrl_t*)&pc_recv_mesg;
		uint8_t move_cmd = chassis_ctrl->move_cmd;
		int16_t vx = chassis_ctrl->x_spd;
		int16_t vy = chassis_ctrl->y_spd;
		float vw = chassis_ctrl->w_info.w_spd;
		
		if (move_cmd != SPD_CTRL) {
			/* Encoder operation */
			chassis.enc_exec_done = 0;
			if (move_cmd == ENC_CTRL_X) {
				chassis_enc_move_mm_x(vx);
			} else if (move_cmd == ENC_CTRL_Y) {
				chassis_enc_move_mm_y(vy);
			} else if (move_cmd == ENC_CTRL_W) {
				chassis_enc_turn_deg(vw);
			}
			chassis_ctrl->move_cmd = SPD_CTRL;
			chassis.enc_exec_done = 1;
		} else {
			// move_cmd == SPD_CTRL
			/* Speed command */
			int16_t speed[4];
			
			mecanum_calc(vx, vy, vw, speed);
			
			runSpeed((float)-speed[0], 1, 0);
			runSpeed((float)-speed[1], 1, 1);
			runSpeed((float)-speed[2], 1, 2);
			runSpeed((float)-speed[3], 1, 3);
			
			/* Reset enc exec if vel command is received */
			chassis.enc_exec_done = 1;
		}

		osDelayUntil(&chassis_wake_time, 100);  // 10Hz
	}
}
