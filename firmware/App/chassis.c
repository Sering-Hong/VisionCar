/**
 * @file    chassis.c
 * @brief   底盘运动控制实现：差速运动学 + 双闭环 PID
 */

#include "chassis.h"
#include "Motor.h"

/* ---- 内部辅助函数 ---- */

static void Chassis_DecomposeVelocity(Chassis_t *c)
{
    /*
     * 差速运动学分解：
     *   v_left  = linear - angular * (wheelbase / 2)
     *   v_right = linear + angular * (wheelbase / 2)
     *
     * 角速度约定：正 = 左转（逆时针），此时右轮快于左轮
     */
    float half_base = CHASSIS_WHEEL_BASE / 2.0f;
    c->target_left  = c->linear_speed - c->angular_speed * half_base;
    c->target_right = c->linear_speed + c->angular_speed * half_base;
}

/* ---- 公共接口 ---- */

void Chassis_Init(Chassis_t *c)
{
    /* 左轮 PID */
    PID_Init(&c->pid_left,  0.20f, 0.27f, 0.01f);
    PID_SetLimits(&c->pid_left, 500.0f, 100.0f);

    /* 右轮 PID（参数略不同以补偿机械差异） */
    PID_Init(&c->pid_right, 0.30f, 0.21f, 0.01f);
    PID_SetLimits(&c->pid_right, 500.0f, 100.0f);

    c->mode = CHASSIS_MODE_STOP;
    c->linear_speed = 0.0f;
    c->angular_speed = 0.0f;
    c->target_left = 0.0f;
    c->target_right = 0.0f;
}

void Chassis_SetVelocity(Chassis_t *c, float linear_speed, float angular_speed)
{
    c->mode = CHASSIS_MODE_VELOCITY;
    c->linear_speed = linear_speed;
    c->angular_speed = angular_speed;
    Chassis_DecomposeVelocity(c);

    PID_SetTarget(&c->pid_left,  c->target_left);
    PID_SetTarget(&c->pid_right, c->target_right);
}

void Chassis_SetRawPWM(Chassis_t *c, float left, float right)
{
    c->mode = CHASSIS_MODE_RAW_PWM;
    Motor_SetPWM(1, (int16_t)left);
    Motor_SetPWM(2, (int16_t)right);
}

void Chassis_Stop(Chassis_t *c)
{
    c->mode = CHASSIS_MODE_STOP;
    c->linear_speed = 0.0f;
    c->angular_speed = 0.0f;
    c->target_left = 0.0f;
    c->target_right = 0.0f;
    PID_Reset(&c->pid_left);
    PID_Reset(&c->pid_right);
    Motor_SetPWM(1, 0);
    Motor_SetPWM(2, 0);
}

void Chassis_Update(Chassis_t *c, int16_t encoder_left, int16_t encoder_right)
{
    if (c->mode == CHASSIS_MODE_STOP || c->mode == CHASSIS_MODE_RAW_PWM) {
        return;
    }

    /* 编码器值转速度 */
    float speed_left  = (float)encoder_left;
    float speed_right = (float)(-encoder_right);  /* 右轮编码器方向取反 */

    /* PID 计算 */
    float pwm_left  = PID_Update(&c->pid_left,  speed_left);
    float pwm_right = PID_Update(&c->pid_right, speed_right);

    /* 输出到电机 */
    Motor_SetPWM(1, (int16_t)pwm_left);
    Motor_SetPWM(2, (int16_t)pwm_right);

    /* 更新状态快照 */
    c->status.left_speed  = speed_left;
    c->status.right_speed = speed_right;
    c->status.left_pwm    = pwm_left;
    c->status.right_pwm   = pwm_right;
    c->status.linear_speed  = c->linear_speed;
    c->status.angular_speed = c->angular_speed;
}

Chassis_Status_t Chassis_GetStatus(Chassis_t *c)
{
    return c->status;
}
