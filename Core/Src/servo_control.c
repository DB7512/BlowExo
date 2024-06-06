#include "servo_control.h"
#include "pid_control.h"
uint8_t servo_send_buf[SERVO_SEND_LEN] = {0};           // �ŷ����ջ���
uint8_t servo_recv_buf[SERVO_RECV_LEN] = {0};           // �ŷ����ͻ���

int target_position = 0;        // �ؽ�Ŀ��λ��
int target_velocity = 0;        // �ؽ�Ŀ���ٶ�
int target_torque = 0;          // �ؽ�Ŀ������
int actual_position = 0;        // ��ǰλ��
int actual_velocity = 0;        // ��ǰ�ٶ�
int actual_torque = 0;        // ��ǰ����
int blow_position = 0;      // ��ǰ������λ��
uint8_t servo_mode = 0;     // �ŷ�����ģʽ
uint8_t motor_mode = 0;     // �������ģʽ
uint8_t servo_error = 0;    // �ŷ�����

/**
 * 
*/
void Servo_Init(MOTOR_send *pData, MOTOR_recv *rData)
{
  // ��ʼ��PID
  Init_PID(&pid);
  // ��ȡ�����Ϣ
  Motor_Control(&cmd, 0, 0, 0.0);
  Motor_Send_Recv(pData, rData);
  // ��ȡ��������Ϣ
  Encoder_Send_Recv(encoder_recv_buf);
}

/* �������ָ��ṹ�帳ֵ
 * mMode: �������ģʽ
 * cMode: �������ģʽ��0���ٶ�ģʽ��1��λ��ģʽ��2������ģʽ��
 */
void Motor_Control(MOTOR_send *cmd, int mMode, int cMode, float cPara)
{
  cmd->id = 0;
  cmd->mode = mMode;
  if(mMode == 0)
  {
    cmd->T = 0;
    cmd->W = 0;
    cmd->Pos = 0;
    cmd->K_P = 0;
    cmd->K_W = 0;
  }
  else if(mMode == 1)
  {
    if(cMode == 0)          // �ٶ�ģʽ��T��K_P����Ϊ0
    {
      cmd->T = 0;
      cmd->W = cPara;
      cmd->Pos = 0;
      cmd->K_P = 0;
      cmd->K_W = 0.05;
    }
    else if(cMode == 1)     // λ��ģʽ
    {
      cmd->T = 0;
      cmd->W = 0;
      cmd->Pos = cPara;
      cmd->K_P = 0.1;
      cmd->K_W = 0.01;
    }
  }
}

void Extract_Servo_Recv(uint8_t *pData)   // , int *position, uint8_t *mode, int *speed, int *torque
{

    /* ��������ģʽ */
    uint8_t mode = pData[2];
    motor_mode = mode >> 4;     // �������ģʽhigh 4 bits
    servo_mode = mode & 0x0F;   // �ŷ�����ģʽlow 4bits

    /* ��������λ�� */
    uint16_t position = 0;
    position |= pData[4];
    position |= pData[3] << 8;
    target_position = position;

    /* ���������ٶ� */
    uint16_t speed = 0;
    speed |= pData[6];
    speed |= pData[5] << 8;
    target_velocity = speed;

    /* ������������ */
    uint16_t torque = 0;
    torque |= pData[8];
    torque |= pData[7] << 8;
    target_torque = torque;

}

void Motor_Send_Recv(MOTOR_send *pData, MOTOR_recv *rData)
{
  modify_data(&cmd);
  // ����
  SET_485_DE_UP();
  SET_485_RE_UP();
  HAL_UART_Transmit(&huart1, (uint8_t *)pData, sizeof(pData->motor_send_data), 10); 
  // ����
  SET_485_RE_DOWN();
  SET_485_DE_DOWN();
  HAL_UART_Receive_IT(&huart1, (uint8_t *)rData, sizeof(rData->motor_recv_data));
}

void Get_Servo_Information()
{
  // ��ȡ�����Ϣ
  Motor_Control(&cmd, 1, 0, 0.0);
  Motor_Send_Recv(&cmd, &data);
  // ��ȡ��������Ϣ
  Encoder_Send_Recv(encoder_recv_buf);
}
