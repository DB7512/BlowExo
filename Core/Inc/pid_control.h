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
    // ���� PID���Ʋ���
    float kp;               // ����ϵ��
    float ki;               // ����ϵ��
    float kd;               // ΢��ϵ��
    float error;            // ���
    float p_out;            // ������
    float i_out;            // ������
    float d_out;            // ΢����
    float last_error;       // �ϴ����
    float integral;         // ����
    float maxIntegral;      // �����޷�
    float output;           // ���
    float maxOutput;        // ����޷�
    float minOutput;
} Motor_PID;     //��������������ݰ�

extern void Init_PID(Motor_PID *pid);
extern void PID_Realize(int target, int actual, float dt, Motor_PID *pid);
extern float Omega_Pluse_to_Radian(int pluse);

#endif
