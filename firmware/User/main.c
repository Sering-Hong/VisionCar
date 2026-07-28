/**
 * @file    main.c
 * @brief   智能小车主控程序（FreeRTOS 多任务架构）
 *
 * @details 系统架构：
 *          - ControlTask  : 10ms 周期，读编码器 + 更新底盘 PID 控制
 *          - CommandTask  : 阻塞等待串口命令，分发到回调函数
 *          - DisplayTask  : 100ms 周期，OLED 刷新系统状态
 *          - defaultTask  : 心跳 + 看门狗
 *
 *          通信协议：上位机发送 @CMD\r\n 格式命令
 *          控制链路：Serial RX ISR -> 命令队列 -> CommandTask -> Chassis/Servo
 */

#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "OLED.h"
#include "Encoder.h"
#include "Motor.h"
#include "Serial.h"
#include "Servo.h"
#include "PWM.h"

#include "pid.h"
#include "chassis.h"
#include "cmd_dispatch.h"

/* ==================== FreeRTOS 任务配置 ==================== */

#define TASK_CONTROL_STACK     (128)   /* 512 bytes */
#define TASK_CONTROL_PRIO      (configMAX_PRIORITIES - 2)
#define TASK_COMMAND_STACK     (128)
#define TASK_COMMAND_PRIO      (configMAX_PRIORITIES - 3)
#define TASK_DISPLAY_STACK     (128)
#define TASK_DISPLAY_PRIO      (configMAX_PRIORITIES - 4)
#define TASK_DEFAULT_STACK     (64)
#define TASK_DEFAULT_PRIO      (1)

/* ==================== 全局对象 ==================== */

static Chassis_t        g_chassis;        /* 底盘控制器 */
static System_Context_t g_sys_ctx;        /* 系统上下文 */

/* FreeRTOS 对象 */
static QueueHandle_t    s_cmd_queue;      /* 命令消息队列 */
static SemaphoreHandle_t s_oled_mutex;    /* OLED 互斥锁 */

/* 命令缓冲区（用于队列传递） */
#define CMD_MAX_LEN  32
typedef struct {
    char data[CMD_MAX_LEN];
} CmdMessage_t;

/* ==================== 任务实现 ==================== */

/**
 * @brief  控制任务 — 10ms 周期读编码器 + 更新底盘 PID
 */
static void Task_Control(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        int16_t enc_left  = Encoder_Get(1);
        int16_t enc_right = Encoder_Get(2);

        Chassis_Update(&g_chassis, enc_left, enc_right);

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    }
}

/**
 * @brief  命令处理任务 — 阻塞等待串口命令队列，分发到回调
 */
static void Task_Command(void *pvParameters)
{
    (void)pvParameters;
    CmdMessage_t msg;

    for (;;) {
        if (xQueueReceive(s_cmd_queue, &msg, portMAX_DELAY) == pdTRUE) {
            CmdDispatcher_Dispatch(msg.data);
        }
    }
}

/**
 * @brief  显示任务 — 100ms 周期刷新 OLED 状态
 *
 * 显示内容：
 *   Line 0: 标题 "RC-Car v2.0"
 *   Line 1: 左轮速度 | 右轮速度
 *   Line 2: 线速度   | 角速度
 *   Line 3: 模式     | 命令计数
 */
static void Task_Display(void *pvParameters)
{
    (void)pvParameters;
    static uint32_t cmd_count = 0;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        Chassis_Status_t st = Chassis_GetStatus(&g_chassis);

        if (xSemaphoreTake(s_oled_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            /* Line 0: 标题 */
            OLED_ShowString(0, 0, "RC-Car v2.0      ");

            /* Line 1: 轮速 */
            char buf[22];
            snprintf(buf, sizeof(buf), "L:%+04.0f R:%+04.0f  ",
                     st.left_speed, st.right_speed);
            OLED_ShowString(0, 2, buf);

            /* Line 2: 线速度/角速度 */
            snprintf(buf, sizeof(buf), "V:%+05.1f W:%+04.2f ",
                     st.linear_speed, st.angular_speed);
            OLED_ShowString(0, 4, buf);

            /* Line 3: 模式 */
            const char *mode_str = (g_chassis.mode == CHASSIS_MODE_STOP)    ? "STOP"
                                 : (g_chassis.mode == CHASSIS_MODE_VELOCITY) ? "VELO"
                                 : "RAW ";
            snprintf(buf, sizeof(buf), "Mode:%s Cnt:%lu ", mode_str,
                     (unsigned long)cmd_count);
            OLED_ShowString(0, 6, buf);

            xSemaphoreGive(s_oled_mutex);
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
    }
}

/**
 * @brief  默认任务 — 心跳指示（板载 LED 闪烁）
 */
static void Task_Default(void *pvParameters)
{
    (void)pvParameters;

    for (;;) {
        /* 心跳：简单翻转 LED 或 nop */
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * @brief  TIM1 中断 — 编码器采样 + 控制周期触发
 *
 * @note   FreeRTOS 模式下不在中断中做 PID 计算，
 *         仅触发控制任务的信号量（或由 vTaskDelayUntil 自动触发）。
 *         此处保留 TIM1 中断用于 FreeRTOS tick 之外的硬件定时。
 */
void TIM1_UP_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET) {
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
        /* 控制逻辑已移至 Task_Control，中断仅清标志 */
    }
}

/* ==================== 串口中断（适配 FreeRTOS） ==================== */

extern char Serial_RxPacket[100];
extern uint8_t Serial_RxFlag;

/**
 * @brief  USART1 中断处理 — 接收命令并投递到队列
 *
 * @note   从 Serial.c 中提取的状态机逻辑在此处直接调用队列，
 *         避免在 ISR 中做 strcmp 分发。
 *         队列投递使用 xQueueSendFromISR（FromISR 安全）。
 */
void USART1_IRQHandler_FreeRTOS(void)
{
    /* 复用 Serial.c 中原有的中断处理逻辑 */
    /* 接收到完整包后，投递到命令队列 */
    if (Serial_RxFlag == 1) {
        CmdMessage_t msg;
        uint8_t len = 0;
        while (Serial_RxPacket[len] != '\0' && len < CMD_MAX_LEN - 1) {
            msg.data[len] = Serial_RxPacket[len];
            len++;
        }
        msg.data[len] = '\0';

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(s_cmd_queue, &msg, &xHigherPriorityTaskWoken);
        Serial_RxFlag = 0;

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/* ==================== 系统初始化 ==================== */

static void System_PeriphInit(void)
{
    /* NVIC 分组：FreeRTOS 要求 Group 4（全抢占） */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    OLED_Init();
    Serial_Init();
    Encoder_Init();
    Motor_Init();
    PWM2_Init();
    Servo_Init ? Servo_Init() : (void)0;  /* 如存在则调用 */
}

static void System_CreateObjects(void)
{
    /* 命令队列：深度 8，每项 32 字节 */
    s_cmd_queue = xQueueCreate(8, sizeof(CmdMessage_t));

    /* OLED 互斥锁 */
    s_oled_mutex = xSemaphoreCreateMutex();

    /* 初始化底盘和命令分发器 */
    Chassis_Init(&g_chassis);
    g_sys_ctx.chassis = &g_chassis;
    g_sys_ctx.servo1_angle = 0;
    g_sys_ctx.servo2_angle = 90;
    CmdDispatcher_Init(&g_sys_ctx);
}

static void System_CreateTasks(void)
{
    xTaskCreate(Task_Control,  "Ctrl",  TASK_CONTROL_STACK,  NULL, TASK_CONTROL_PRIO,  NULL);
    xTaskCreate(Task_Command,  "Cmd",   TASK_COMMAND_STACK,  NULL, TASK_COMMAND_PRIO,  NULL);
    xTaskCreate(Task_Display,  "Disp",  TASK_DISPLAY_STACK,  NULL, TASK_DISPLAY_PRIO,  NULL);
    xTaskCreate(Task_Default,  "Idle",  TASK_DEFAULT_STACK,  NULL, TASK_DEFAULT_PRIO,  NULL);
}

/* ==================== 入口 ==================== */

int main(void)
{
    System_PeriphInit();
    System_CreateObjects();
    System_CreateTasks();

    /* 显示启动画面 */
    if (xSemaphoreTake(s_oled_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        OLED_ShowString(0, 0, "RC-Car v2.0      ");
        OLED_ShowString(0, 2, "FreeRTOS Starting ");
        OLED_ShowString(0, 4, "4 Tasks Ready     ");
        xSemaphoreGive(s_oled_mutex);
    }

    /* 启动调度器 — 此处之后不会返回 */
    vTaskStartScheduler();

    /* 不应到达此处 */
    while (1) {}
}
