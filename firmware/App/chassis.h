/**
 * @file    chassis.h
 * @brief   底盘运动控制模块（双轮差速运动学 + 双闭环 PID）
 *
 * @details 将线速度 (cm/s) + 角速度 (rad/s) 分解为左右轮目标转速，
 *          通过编码器反馈实现双闭环 PID 速度控制。
 *          适用于双轮差速驱动 + 万向轮底盘结构。
 */

#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "pid.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 底盘物理参数（根据实际小车调整） ---- */
#define CHASSIS_WHEEL_BASE      16.0f   /* 轮距 (cm)，左右轮中心距 */
#define CHASSIS_WHEEL_RADIUS    3.25f   /* 轮半径 (cm) */
#define CHASSIS_ENCODER_PPR     13.0f   /* 编码器每转脉冲数（减速前） */
#define CHASSIS_GEAR_RATIO      34.0f   /* 减速比 */
#define CHASSIS_SPEED_SCALE     10.0f   /* PWM 到速度的比例系数（标定值） */

/**
 * @brief  底盘运动模式
 */
typedef enum {
    CHASSIS_MODE_STOP = 0,    /* 停止 */
    CHASSIS_MODE_VELOCITY,    /* 速度闭环控制 */
    CHASSIS_MODE_RAW_PWM,     /* 开环 PWM 直通（调试用） */
} Chassis_Mode_t;

/**
 * @brief  底盘状态信息（用于上位机监控）
 */
typedef struct {
    float left_speed;         /* 左轮实际速度 */
    float right_speed;        /* 右轮实际速度 */
    float left_pwm;           /* 左轮 PID 输出 */
    float right_pwm;          /* 右轮 PID 输出 */
    float linear_speed;       /* 当前设定线速度 */
    float angular_speed;      /* 当前设定角速度 */
} Chassis_Status_t;

/**
 * @brief  底盘控制实例
 */
typedef struct {
    PID_Controller_t pid_left;
    PID_Controller_t pid_right;
    Chassis_Mode_t mode;

    float linear_speed;       /* 设定线速度 (cm/s) */
    float angular_speed;      /* 设定角速度 (rad/s) */

    float target_left;        /* 左轮目标速度 */
    float target_right;       /* 右轮目标速度 */

    Chassis_Status_t status;  /* 状态快照 */
} Chassis_t;

/**
 * @brief  初始化底盘
 * @param  c 底盘实例指针
 */
void Chassis_Init(Chassis_t *c);

/**
 * @brief  设置运动速度（速度闭环模式）
 * @param  c             底盘实例指针
 * @param  linear_speed  线速度 (cm/s)，正=前进
 * @param  angular_speed 角速度 (rad/s)，正=左转
 */
void Chassis_SetVelocity(Chassis_t *c, float linear_speed, float angular_speed);

/**
 * @brief  直接设置左右轮 PWM（开环调试模式）
 * @param  c   底盘实例指针
 * @param  left  左轮 PWM (-100 ~ +100)
 * @param  right 右轮 PWM (-100 ~ +100)
 */
void Chassis_SetRawPWM(Chassis_t *c, float left, float right);

/**
 * @brief  紧急停车
 * @param  c 底盘实例指针
 */
void Chassis_Stop(Chassis_t *c);

/**
 * @brief  底盘控制更新（在定时器中断中调用，典型周期 10ms）
 * @param  c             底盘实例指针
 * @param  encoder_left  左轮编码器读数
 * @param  encoder_right 右轮编码器读数
 */
void Chassis_Update(Chassis_t *c, int16_t encoder_left, int16_t encoder_right);

/**
 * @brief  获取底盘状态快照
 * @param  c 底盘实例指针
 * @return 状态结构体
 */
Chassis_Status_t Chassis_GetStatus(Chassis_t *c);

#ifdef __cplusplus
}
#endif

#endif /* __CHASSIS_H */
