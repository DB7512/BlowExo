#ifndef __PID_CONTROL_H
#define __PID_CONTROL_H

#include <stdint.h>
#include "main.h" // stm32 hal

/**
 * @brief PID
 * 
 */
typedef struct
{
    // 定义 PID控制参数
    float kp;               // 比例系数
    float ki;               // 积分系数
    float kd;               // 微分系数
    float error;            // 误差
    float p_out;            // 比例项
    float i_out;            // 积分项
    float d_out;            // 微分项
    float last_error;       // 上次误差
    float integral;         // 积分
    float maxIntegral;      // 积分限幅
    float output;           // 输出
    float maxOutput;        // 输出限幅
    float minOutput;
} Motor_PID;     //电机控制命令数据包

extern void Init_PID(Motor_PID *pid);
extern void PID_Realize(int target, int actual, float dt, Motor_PID *pid);
extern float Omega_Pluse_to_Radian(int pluse);

#endif
