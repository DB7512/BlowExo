#include "pid_control.h"

void Init_PID(Motor_PID *pid)
{
  pid->kp = 0;               // ����ϵ��
  pid->ki = 0;               // ����ϵ��
  pid->kd = 0;               // ΢��ϵ��
  pid->error = 0;            // ���
  pid->p_out = 0;            // ������
  pid->i_out = 0;            // ������
  pid->d_out = 0;            // ΢����
  pid->last_error = 0;       // �ϴ����
  pid->integral = 0;         // ����
  pid->maxIntegral = 0;      // �����޷�
  pid->output = 0;           // ���
  pid->maxOutput = 312911;   // ����޷�(30rad/sת��������ֵ30/2/pi*65536)
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

/** ���������ٶȵ�����ֵת���ɵ��ת���ٶȵĻ���ֵ��ֱ�Ӹ�ֵ��MOTOR_send->W
 ** pluse->rad
 */
float Omega_Pluse_to_Radian(int pluse)
{
  float omega = 0.0;
  omega = pluse * 6.2832f / 65536 / 6.33f;
  return omega;
}
