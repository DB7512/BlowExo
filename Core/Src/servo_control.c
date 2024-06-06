#include "servo_control.h"
#include "pid_control.h"
uint8_t servo_send_buf[SERVO_SEND_LEN] = {0};           // 伺服接收缓存
uint8_t servo_recv_buf[SERVO_RECV_LEN] = {0};           // 伺服发送缓存

int target_position = 0;        // 关节目标位置
int target_velocity = 0;        // 关节目标速度
int target_torque = 0;          // 关节目标力矩
int actual_position = 0;        // 当前位置
int actual_velocity = 0;        // 当前速度
int actual_torque = 0;        // 当前力矩
int blow_position = 0;      // 当前编码器位置
uint8_t servo_mode = 0;     // 伺服控制模式
uint8_t motor_mode = 0;     // 电机工作模式
uint8_t servo_error = 0;    // 伺服错误

/**
 * 
*/
void Servo_Init(MOTOR_send *pData, MOTOR_recv *rData)
{
  // 初始化PID
  Init_PID(&pid);
  // 获取电机信息
  Motor_Control(&cmd, 0, 0, 0.0);
  Motor_Send_Recv(pData, rData);
  // 获取编码器信息
  Encoder_Send_Recv(encoder_recv_buf);
}

/* 电机控制指令结构体赋值
 * mMode: 电机工作模式
 * cMode: 电机控制模式（0：速度模式，1：位置模式，2：力矩模式）
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
    if(cMode == 0)          // 速度模式，T和K_P必须为0
    {
      cmd->T = 0;
      cmd->W = cPara;
      cmd->Pos = 0;
      cmd->K_P = 0;
      cmd->K_W = 0.05;
    }
    else if(cMode == 1)     // 位置模式
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

    /* 解析工作模式 */
    uint8_t mode = pData[2];
    motor_mode = mode >> 4;     // 电机工作模式high 4 bits
    servo_mode = mode & 0x0F;   // 伺服控制模式low 4bits

    /* 解析期望位置 */
    uint16_t position = 0;
    position |= pData[4];
    position |= pData[3] << 8;
    target_position = position;

    /* 解析期望速度 */
    uint16_t speed = 0;
    speed |= pData[6];
    speed |= pData[5] << 8;
    target_velocity = speed;

    /* 解析期望力矩 */
    uint16_t torque = 0;
    torque |= pData[8];
    torque |= pData[7] << 8;
    target_torque = torque;

}

void Motor_Send_Recv(MOTOR_send *pData, MOTOR_recv *rData)
{
  modify_data(&cmd);
  // 发送
  SET_485_DE_UP();
  SET_485_RE_UP();
  HAL_UART_Transmit(&huart1, (uint8_t *)pData, sizeof(pData->motor_send_data), 10); 
  // 接收
  SET_485_RE_DOWN();
  SET_485_DE_DOWN();
  HAL_UART_Receive_IT(&huart1, (uint8_t *)rData, sizeof(rData->motor_recv_data));
}

void Get_Servo_Information()
{
  // 获取电机信息
  Motor_Control(&cmd, 1, 0, 0.0);
  Motor_Send_Recv(&cmd, &data);
  // 获取编码器信息
  Encoder_Send_Recv(encoder_recv_buf);
}
