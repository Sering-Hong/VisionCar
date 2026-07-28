/**
 * @file    pid.h
 * @brief   增量式/位置式 PID 控制器抽象层
 *
 * @details 支持积分限幅、输出限幅、微分先行（可选），
 *          可在定时器中断中安全调用 PID_Update()。
 */

#ifndef __PID_H
#define __PID_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  PID 控制器实例
 */
typedef struct {
    /* 增益参数 */
    float Kp;
    float Ki;
    float Kd;

    /* 内部状态 */
    float target;
    float last_error;
    float integral;
    float last_measurement;  /* 微分先行用 */

    /* 限幅 */
    float integral_limit;    /* 积分抗饱和限幅 */
    float output_limit;      /* 输出限幅 */

    /* 输出 */
    float output;
} PID_Controller_t;

/**
 * @brief  初始化 PID 控制器
 * @param  pid   控制器实例指针
 * @param  kp    比例增益
 * @param  ki    积分增益
 * @param  kd    微分增益
 */
void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd);

/**
 * @brief  设置积分和输出限幅
 * @param  pid            控制器实例指针
 * @param  integral_limit 积分限幅值（正值）
 * @param  output_limit   输出限幅值（正值）
 */
void PID_SetLimits(PID_Controller_t *pid, float integral_limit, float output_limit);

/**
 * @brief  设置目标值
 * @param  pid    控制器实例指针
 * @param  target 目标值
 */
void PID_SetTarget(PID_Controller_t *pid, float target);

/**
 * @brief  重置控制器状态（积分清零等）
 * @param  pid 控制器实例指针
 */
void PID_Reset(PID_Controller_t *pid);

/**
 * @brief  位置式 PID 更新（在定时器中断中调用）
 * @param  pid          控制器实例指针
 * @param  measurement  当前测量值
 * @return 控制输出
 */
float PID_Update(PID_Controller_t *pid, float measurement);

/**
 * @brief  增量式 PID 更新（适合执行器无积分特性场景）
 * @param  pid          控制器实例指针
 * @param  measurement  当前测量值
 * @return 控制增量
 */
float PID_UpdateIncremental(PID_Controller_t *pid, float measurement);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */
