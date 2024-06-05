#include "pid_control.h"

void Init_PID(Motor_PID *pid)
{
  pid->kp = 0;               // 比例系数
  pid->ki = 0;               // 积分系数
  pid->kd = 0;               // 微分系数
  pid->error = 0;            // 误差
  pid->p_out = 0;            // 比例项
  pid->i_out = 0;            // 积分项
  pid->d_out = 0;            // 微分项
  pid->last_error = 0;       // 上次误差
  pid->integral = 0;         // 积分
  pid->maxIntegral = 0;      // 积分限幅
  pid->output = 0;           // 输出
  pid->maxOutput = 312911;   // 输出限幅(30rad/s转换成脉冲值30/2/pi*65536)
  pid->minOutput = -312911;
}

void PID_Realize(int target, int actual, float dt, Motor_PID *pid)
{
  pid->error = target - actual;
  
  pid->p_out = pid->kp * pid->error;
  
  pid->integral += pid->error * dt;
  pid->i_out = pid->ki * pid->integral;
  
  pid->d_out = pid->kd * (pid->error - pid->last_error) / dt;
  
  pid->output = pid->p_out + pid->i_out + pid->d_out;
  
  pid->last_error = pid->error;
  
  if(pid->output > pid->maxOutput)
  {
    pid->output = pid->maxOutput;
  }
  else if(pid->output < pid->minOutput)
  {
    pid->output = pid->minOutput;
  }
}

/** 电机输出端速度的脉冲值转换成电机转子速度的弧度值，直接赋值给MOTOR_send->W
 ** pluse->rad
 */
float Omega_Pluse_to_Radian(int pluse)
{
  float omega = 0.0;
  omega = pluse * 6.2832f / 65536 / 6.33f;
  return omega;
}
