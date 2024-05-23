#include "pid_control.h"

void PID_Realize(float target, float actual, float dt, Motor_PID *pid)
{
  pid->error = target - actual;
  
  pid->p_out = pid->kp * pid->error;
  
  pid->integral += pid->error * dt;
  pid->i_out = pid->ki * pid->integral;
  
  pid->d_out = pid->kd * (pid->error - pid->last_error) / dt;
  
  pid->output = pid->p_out + pid->i_out + pid->d_out;
  
  pid->last_error = pid->error;
}
