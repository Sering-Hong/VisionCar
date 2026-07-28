/**
 * @file    pid.c
 * @brief   PID 控制器实现
 */

#include "pid.h"

void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->target = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->last_measurement = 0.0f;
    pid->integral_limit = 500.0f;
    pid->output_limit = 100.0f;
    pid->output = 0.0f;
}

void PID_SetLimits(PID_Controller_t *pid, float integral_limit, float output_limit)
{
    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;
}

void PID_SetTarget(PID_Controller_t *pid, float target)
{
    pid->target = target;
}

void PID_Reset(PID_Controller_t *pid)
{
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->last_measurement = 0.0f;
    pid->output = 0.0f;
}

static float Clamp(float value, float limit)
{
    if (value > limit)  return limit;
    if (value < -limit) return -limit;
    return value;
}

float PID_Update(PID_Controller_t *pid, float measurement)
{
    float error = pid->target - measurement;

    /* 积分累加 + 抗饱和 */
    pid->integral += error;
    pid->integral = Clamp(pid->integral, pid->integral_limit);

    /* 微分项（基于测量值微分，避免目标突变引起的微分尖刺） */
    float derivative = measurement - pid->last_measurement;

    /* 位置式 PID 输出 */
    float output = pid->Kp * error
                 + pid->Ki * pid->integral
                 + pid->Kd * (-derivative);  /* 取负号因为用的是 measurement 微分 */

    /* 输出限幅 */
    output = Clamp(output, pid->output_limit);

    /* 更新历史状态 */
    pid->last_error = error;
    pid->last_measurement = measurement;
    pid->output = output;

    return output;
}

float PID_UpdateIncremental(PID_Controller_t *pid, float measurement)
{
    float error = pid->target - measurement;

    /* 增量计算 */
    float delta = pid->Kp * (error - pid->last_error)
                + pid->Ki * error
                + pid->Kd * (error - 2.0f * pid->last_error + pid->last_measurement);

    pid->output += delta;
    pid->output = Clamp(pid->output, pid->output_limit);

    pid->last_measurement = pid->last_error;
    pid->last_error = error;

    return pid->output;
}
