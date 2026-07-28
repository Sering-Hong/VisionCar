/**
 * @file    cmd_dispatch.h
 * @brief   串口命令分发表（查找表替代 if-else 链）
 */

#ifndef __CMD_DISPATCH_H
#define __CMD_DISPATCH_H

#include "chassis.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  系统上下文（供命令回调使用）
 */
typedef struct {
    Chassis_t *chassis;
    uint8_t    servo1_angle;
    uint8_t    servo2_angle;
} System_Context_t;

/**
 * @brief  初始化命令分发器
 * @param  ctx 系统上下文指针
 */
void CmdDispatcher_Init(System_Context_t *ctx);

/**
 * @brief  分发一条命令字符串
 * @param  cmd 命令字符串（不含 @ 前缀和 \r\n 后缀）
 */
void CmdDispatcher_Dispatch(const char *cmd);

#ifdef __cplusplus
}
#endif

#endif /* __CMD_DISPATCH_H */
