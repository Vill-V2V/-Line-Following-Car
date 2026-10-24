#include "huidu.h"
#include "motor.h"

// 保存上一次有效的位置偏差，用于 PD 微分控制(D项) 以及 极速脱轨时的方向记忆
static float last_error = 0.0f;

/**
 * @brief 严格匹配 SysConfig 生成宏的 8 路灰度读取函数
 * @return 返回 8-bit 数据，Bit0对应HD1，Bit7对应HD8。置1表示触发黑线
 */
uint8_t Huidu_Read_Raw(void)
{
    uint8_t sensor_data = 0;

    // HD1: GPIOA PIN_22
    if ((DL_GPIO_readPins(HUIDU_l1_PORT, HD_HD1_PIN) ? 1 : 0) == HUIDU_BLACK_LEVEL) sensor_data |= (1 << 0);
    // HD2: GPIOB PIN_6
    if ((DL_GPIO_readPins(HD_HD2_PORT, HD_HD2_PIN) ? 1 : 0) == HUIDU_BLACK_LEVEL) sensor_data |= (1 << 1);
    // HD3: GPIOA PIN_18
    if ((DL_GPIO_readPins(HD_HD3_PORT, HD_HD3_PIN) ? 1 : 0) == HUIDU_BLACK_LEVEL) sensor_data |= (1 << 2);
    // HD4: GPIOB PIN_15
    if ((DL_GPIO_readPins(HD_HD4_PORT, HD_HD4_PIN) ? 1 : 0) == HUIDU_BLACK_LEVEL) sensor_data |= (1 << 3);
    // HD5: GPIOB PIN_16
    if ((DL_GPIO_readPins(HD_HD5_PORT, HD_HD5_PIN) ? 1 : 0) == HUIDU_BLACK_LEVEL) sensor_data |= (1 << 4);
    // HD6: GPIOB PIN_7
    if ((DL_GPIO_readPins(HD_HD6_PORT, HD_HD6_PIN) ? 1 : 0) == HUIDU_BLACK_LEVEL) sensor_data |= (1 << 5);
    // HD7: GPIOA PIN_17
    if ((DL_GPIO_readPins(HD_HD7_PORT, HD_HD7_PIN) ? 1 : 0) == HUIDU_BLACK_LEVEL) sensor_data |= (1 << 6);
    // HD8: GPIOA PIN_16
    if ((DL_GPIO_readPins(HD_HD8_PORT, HD_HD8_PIN) ? 1 : 0) == HUIDU_BLACK_LEVEL) sensor_data |= (1 << 7);

    return sensor_data;
}

/**
 * @brief 加权位置偏差算法 (带脱轨记忆)
 * @return 负值表示黑线偏左（需左转），正值表示黑线偏右（需右转），0表示居中
 */
float Huidu_Get_Error(void)
{
    uint8_t raw = Huidu_Read_Raw();
    
    // 8个探头的加权分布：HD1(-7) ~ HD8(+7)
    const float weights[8] = {-7.0f, -5.0f, -3.0f, -1.0f, 1.0f, 3.0f, 5.0f, 7.0f};
    
    float sum_weight = 0.0f;
    uint8_t active_count = 0;

    for (int i = 0; i < 8; i++) {
        if (raw & (1 << i)) {
            sum_weight += weights[i];
            active_count++;
        }
    }

    // 1. 全白/脱轨处理逻辑：保持上一次的方向强行恢复线内，防止冲出赛道乱跑
    if (active_count == 0) {
        if (last_error > 0) return 8.0f;    // 极右脱轨，返回最大偏向强制右转
        else if (last_error < 0) return -8.0f; // 极左脱轨，返回最大偏向强制左转
        else return 0.0f;
    }

    // 2. 正常加权计算平均中心偏差
    float current_error = sum_weight / active_count;
    last_error = current_error; // 刷新上一次有效误差

    return current_error;
}

/**
 * @brief 比例-微分 (PD) 循迹核心逻辑
 * @param base_speed 小车直行基础 PWM (如 2500)
 * @param kp 比例增益 (修正当前偏离的大小)
 * @param kd 微分增益 (抑制小车高姿态下的蛇形震荡)
 */
void Huidu_Track_PD(int16_t base_speed, float kp, float kd)
{
    float error = Huidu_Get_Error();
    
    // 计算 PD 转向修正量
    float derivative = error - last_error; 
    int16_t turn_output = (int16_t)(error * kp + derivative * kd);

    // 计算双侧轮速 (以差速形式叠加)
    int16_t left_speed  = base_speed + turn_output;
    int16_t right_speed = base_speed - turn_output;

    // 驱动电机输出 (直接调用 motor.c 中的接口)
    Motor_SetSpeed(left_speed, right_speed);
}