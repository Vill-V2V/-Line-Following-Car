#ifndef MOTOR_H
#define MOTOR_H
#include "ti_msp_dl_config.h"

void Motor_Init(void);
void Motor_Set_Dir(uint8_t ch, uint8_t dir);
void Motor_Set_Duty(uint8_t ch, int16_t duty);
void Motor_Set_Speed(int16_t left, int16_t right);
void Motor_Stop(void);

#endif