// /*
//  * Copyright (c) 2021, Texas Instruments Incorporated
//  * All rights reserved.
//  *
//  * Redistribution and use in source and binary forms, with or without
//  * modification, are permitted provided that the following conditions
//  * are met:
//  *
//  * *  Redistributions of source code must retain the above copyright
//  *    notice, this list of conditions and the following disclaimer.
//  *
//  * *  Redistributions in binary form must reproduce the above copyright
//  *    notice, this list of conditions and the following disclaimer in the
//  *    documentation and/or other materials provided with the distribution.
//  *
//  * *  Neither the name of Texas Instruments Incorporated nor the names of
//  *    its contributors may be used to endorse or promote products derived
//  *    from this software without specific prior written permission.
//  *
//  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
//  * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
//  * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
//  * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
//  * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
//  * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
//  * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
//  * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
//  * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
//  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//  */

#include "ti_msp_dl_config.h"
#include "delay.h"
#include "motor.h"
#include "huidu.h"
#include "oled.h"
#include <stdio.h>
#include <stdint.h>

// 循迹PD参数（8路灰度专用，可现场微调）
#define BASE_SPEED 400     // 基础直行PWM 0~4000
#define KP 45.0f           // 比例系数，越大转弯越灵敏
#define KD 20.0f           // 微分，抑制小车左右抖动
#define PWM_MAX 1000       // PWM上限，防止满速失控
#define PWM_MIN 0          // PWM下限，最低停机

// 串口调试缓存
char debug_buf[128];

int main(void)
{
    SYSCFG_DL_init();     // SysConfig自动初始化全部外设
    Motor_Init();         // 电机PWM+方向初始化
    OLED_Init();          // OLED屏幕初始化
    DL_Timer_startCounter(SERVO_INST); // 启动舵机定时器
    
    // 屏幕初始显示
    OLED_Clear();
    OLED_ShowString(1, 1, "8CH Track PD Car", 16);

    Huidu_Status_t huidu;
    int16_t last_error = 0;
    int16_t turn_output = 0;
    int16_t left_pwm = 0;
    int16_t right_pwm = 0;

    // 上电先停机1s，避免开机瞬间冲出去
    Motor_Stop();
    delay_ms(1000);

    while (1) 
    {
        // 1、读取8路灰度数据，计算偏移误差error、踩线数量line_cnt
        huidu = Huidu_Read();
        int16_t error = huidu.error;
        int16_t d_error = error - last_error;

        // 2、PD差速计算
        turn_output = (int16_t)(KP * error + KD * d_error);
        last_error = error;

        // 3、左右轮速度分配
        left_pwm  = BASE_SPEED + turn_output;
        right_pwm = BASE_SPEED - turn_output;

        // 4、PWM限幅（解决电机不受控一直满转核心逻辑）
        if(left_pwm > PWM_MAX) left_pwm = PWM_MAX;
        if(left_pwm < PWM_MIN) left_pwm = PWM_MIN;
        if(right_pwm > PWM_MAX) right_pwm = PWM_MAX;
        if(right_pwm < PWM_MIN) right_pwm = PWM_MIN;

        // 5、脱线保护：8路全部无黑线直接停机
        if (huidu.line_cnt == 0) 
        {
            Motor_Stop();
            OLED_ShowString(2, 1, "Status: LOST LINE", 16);
        } 
        else 
        {
            // 下发速度到硬件PWM，变量与硬件同步
            Motor_Set_Speed(left_pwm, right_pwm);
            OLED_ShowString(2, 1, "Status: RUNNING  ",4,16);
        }

        // 6、OLED屏幕打印调试信息
        OLED_ShowString(3, 1, "Err:", 4,16);
        OLED_ShowSignedNum(3, 30, error, 4,16);
        OLED_ShowString(4, 1, "L:", 4,16);
        OLED_ShowNum(4, 15, "left_pwm", 4,16);
        OLED_ShowString(4, 40, "R:", 4,16);
        OLED_ShowNum(4, 55, right_pwm, 4,16);

        // 7、串口实时打印（方便电脑调试）
        sprintf(debug_buf, "Err:%d Turn:%d L_PWM:%d R_PWM:%d LineCnt:%d\r\n",
                error, turn_output, left_pwm, right_pwm, huidu.line_cnt);
        UART_SendString(DEBUG_INST, debug_buf);

        // 控制周期10ms，稳定采样
        delay_ms(10);
    }
}