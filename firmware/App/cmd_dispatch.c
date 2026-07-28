/**
 * @file    cmd_dispatch.c
 * @brief   串口命令分发实现：查找表 + 回调函数
 */

#include "cmd_dispatch.h"
#include "Servo.h"
#include <string.h>

/* ---- 静态上下文 ---- */
static System_Context_t *s_ctx;

/* ---- 命令回调函数 ---- */

static void Cmd_Forward(void)     { Chassis_SetVelocity(s_ctx->chassis, -40.0f, 0.0f); }
static void Cmd_Back(void)        { Chassis_SetVelocity(s_ctx->chassis,  40.0f, 0.0f); }
static void Cmd_AvoidForward(void){ Chassis_SetVelocity(s_ctx->chassis, -30.0f, 0.0f); }
static void Cmd_TurnLeft(void)    { Chassis_SetVelocity(s_ctx->chassis, -10.0f, -1.5f); }
static void Cmd_TurnRight(void)   { Chassis_SetVelocity(s_ctx->chassis, -10.0f,  1.5f); }
static void Cmd_AvoidLeft(void)   { Chassis_SetVelocity(s_ctx->chassis,   0.0f, -2.5f); }
static void Cmd_AvoidRight(void)  { Chassis_SetVelocity(s_ctx->chassis,   0.0f,  2.5f); }
static void Cmd_LittleLeft(void)  { Chassis_SetVelocity(s_ctx->chassis, -17.5f, -0.3f); }
static void Cmd_LittleRight(void) { Chassis_SetVelocity(s_ctx->chassis, -17.5f,  0.3f); }
static void Cmd_Stop(void)        { Chassis_Stop(s_ctx->chassis); }

static void Cmd_ServoOpen(void)   { Servo_SetAngle1(65); s_ctx->servo1_angle = 65; }
static void Cmd_ServoGrab(void)   { Servo_SetAngle1(10); s_ctx->servo1_angle = 10; }
static void Cmd_ServoTurn(void)   { Servo_SetAngle2(30); s_ctx->servo2_angle = 30; }
static void Cmd_ServoReturn(void) { Servo_SetAngle2(90); s_ctx->servo2_angle = 90; }

/* ---- 命令查找表 ---- */

typedef struct {
    const char *cmd;
    void (*handler)(void);
} CmdEntry_t;

static const CmdEntry_t s_cmd_table[] = {
    /* 运动控制 */
    {"FORWARD",       Cmd_Forward},
    {"BACK",          Cmd_Back},
    {"LEFT",          Cmd_TurnLeft},
    {"RIGHT",         Cmd_TurnRight},
    {"AVOID_FORWARD", Cmd_AvoidForward},
    {"AVOID_BACK",    Cmd_Back},
    {"AVOID_LEFT",    Cmd_AvoidLeft},
    {"AVOID_RIGHT",   Cmd_AvoidRight},
    {"LITTLELEFT",    Cmd_LittleLeft},
    {"LITTLERIGHT",   Cmd_LittleRight},
    {"STOP",          Cmd_Stop},
    {"AVOID_STOP",    Cmd_Stop},

    /* 机械臂控制 */
    {"BLUE_OPEN",     Cmd_ServoOpen},
    {"BLUE_GRAB",     Cmd_ServoGrab},
    {"BLUE_TURN",     Cmd_ServoTurn},
    {"BLUE_RETURN",   Cmd_ServoReturn},

    /* 哨兵 */
    {NULL, NULL}
};

/* ---- 公共接口 ---- */

void CmdDispatcher_Init(System_Context_t *ctx)
{
    s_ctx = ctx;
}

void CmdDispatcher_Dispatch(const char *cmd)
{
    for (const CmdEntry_t *entry = s_cmd_table; entry->cmd != NULL; entry++) {
        if (strcmp(cmd, entry->cmd) == 0) {
            entry->handler();
            return;
        }
    }
    /* 未知命令：静默忽略 */
}
