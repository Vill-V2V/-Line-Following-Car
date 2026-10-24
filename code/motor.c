#include "motor.h"

// 假设 PWM 周期自动设定上限为 1000
#define PWM_MAX 1000

void Motor_Init(void)
{
    // 使能L298N STBY引脚，解除休眠
    DL_GPIO_setPins(DC_MOTOR_STBY_PORT, DC_MOTOR_STBY_PIN);
    // 启动电机PWM定时器
    DL_Timer_startCounter(PWMAB_INST);
    // 初始占空比0，上电不冲车
    DL_Timer_setCaptureCompareValue(PWMAB_INST, 0, GPIO_PWMAB_C0_IDX);
    DL_Timer_setCaptureCompareValue(PWMAB_INST, 0, GPIO_PWMAB_C1_IDX);
}

// ch=1左轮 ch=2右轮 dir=1前进 dir=0后退
void Motor_Set_Dir(uint8_t ch, uint8_t dir)
{
    if(ch == 1)
    {
        if(dir == 1)
        {
            DL_GPIO_setPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_clearPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        }
        else
        {
            DL_GPIO_clearPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        }
    }
    if(ch == 2)
    {
        if(dir == 1)
        {
            DL_GPIO_setPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
            DL_GPIO_clearPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        }
        else
        {
            DL_GPIO_clearPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        }
    }
}

// 设置单路PWM占空比
void Motor_Set_Duty(uint8_t ch, int16_t duty)
{
    if(ch == 1)
        DL_Timer_setCaptureCompareValue(PWMAB_INST, duty, GPIO_PWMAB_C1_IDX);
    if(ch == 2)
        DL_Timer_setCaptureCompareValue(PWMAB_INST, duty, GPIO_PWMAB_C0_IDX);
}

// 统一设置左右轮速度，默认前进
void Motor_Set_Speed(int16_t left, int16_t right)
{
    Motor_Set_Dir(1, 1);
    Motor_Set_Dir(2, 1);
    Motor_Set_Duty(1, left);
    Motor_Set_Duty(2, right);
}

// 电机全部停机
void Motor_Stop(void)
{
    Motor_Set_Duty(1, 0);
    Motor_Set_Duty(2, 0);
}

 



