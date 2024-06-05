#ifndef __SERVO_CONTROL_H
#define __SERVO_CONTROL_H

#include <stdint.h>
#include "main.h"
#include "usart.h"
#include "RLS_encoder.h"

#define SERVO_SEND_LEN 11
#define SERVO_RECV_LEN 11
#define HEAD 0xFF
#define ADDRESS 0xFE

extern uint8_t servo_send_buf[SERVO_SEND_LEN];
extern uint8_t servo_recv_buf[SERVO_RECV_LEN];

extern int target_position;
extern int target_velocity;        // 关节目标速度
extern int target_torque;          // 关节目标力矩
extern int actual_position;        // 当前位置
extern int actual_velocity;        // 当前速度
extern int actual_torque;          // 当前力矩
extern int blow_position;
extern uint8_t servo_mode;
extern uint8_t motor_mode;
extern uint8_t servo_error;

extern void Servo_Init(MOTOR_send *pData, MOTOR_recv *rData);
extern void Motor_Control(MOTOR_send *cmd, int mMode, int cMode, float cPara);
extern void Extract_Servo_Recv(uint8_t *pData);
extern void Motor_Send_Recv(MOTOR_send *pData, MOTOR_recv *rData);
extern void Get_Servo_Information();

#endif