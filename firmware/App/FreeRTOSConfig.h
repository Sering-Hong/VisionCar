/**
 * @file    FreeRTOSConfig.h
 * @brief   FreeRTOS 配置（STM32F103C8 — Cortex-M3, 72MHz, 20KB RAM）
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* ---- Cortex-M3 特定 ---- */
#define configPRIO_BITS                        4

/* ---- 调度器 ---- */
#define configUSE_PREEMPTION                   1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 0
#define configUSE_TICKLESS_IDLE                0
#define configCPU_CLOCK_HZ                     (72000000UL)
#define configTICK_RATE_HZ                     (1000)
#define configMAX_PRIORITIES                   8
#define configMINIMAL_STACK_SIZE               (64)
#define configMAX_TASK_NAME_LEN                12
#define configUSE_16_BIT_TICKS                 0
#define configIDLE_SHOULD_YIELD                1

/* ---- 内存 ---- */
#define configTOTAL_HEAP_SIZE                  ((size_t)(10 * 1024))  /* 10KB 堆 */
#define configSUPPORT_STATIC_ALLOCATION        0
#define configSUPPORT_DYNAMIC_ALLOCATION       1

/* ---- 钩子函数 ---- */
#define configUSE_IDLE_HOOK                    0
#define configUSE_TICK_HOOK                    0
#define configCHECK_FOR_STACK_OVERFLOW         2  /* 启用栈溢出检测 */
#define configUSE_MALLOC_FAILED_HOOK           1

/* ---- 运行时统计 ---- */
#define configGENERATE_RUN_TIME_STATS          0
#define configUSE_TRACE_FACILITY               0
#define configUSE_STATS_FORMATTING_FUNCTIONS   0

/* ---- 协程（不使用） ---- */
#define configUSE_CO_ROUTINES                  0
#define configMAX_CO_ROUTINE_PRIORITIES        2

/* ---- 软件定时器 ---- */
#define configUSE_TIMERS                       0

/* ---- 互斥锁 / 信号量 ---- */
#define configUSE_MUTEXES                      1
#define configUSE_RECURSIVE_MUTEXES            0
#define configUSE_COUNTING_SEMAPHORES          0
#define configQUEUE_REGISTRY_SIZE              0

/* ---- 中断优先级 ---- */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY        15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY   5
#define configKERNEL_INTERRUPT_PRIORITY     (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

/* ---- API 包含 ---- */
#define INCLUDE_vTaskPrioritySet               1
#define INCLUDE_uxTaskPriorityGet              1
#define INCLUDE_vTaskDelete                    0
#define INCLUDE_vTaskSuspend                   1
#define INCLUDE_xResumeFromISR                 0
#define INCLUDE_vTaskDelayUntil                1
#define INCLUDE_vTaskDelay                     1
#define INCLUDE_xTaskGetSchedulerState         1
#define INCLUDE_xTaskGetCurrentTaskHandle      0
#define INCLUDE_uxTaskGetStackHighWaterMark    1
#define INCLUDE_eTaskGetState                  0
#define INCLUDE_xTimerPendFunctionCall         0
#define INCLUDE_xTaskAbortDelay                0
#define INCLUDE_xTaskGetHandle                 0

/* ---- 断言 ---- */
#define configASSERT(x) if ((x) == 0) { taskDISABLE_INTERRUPTS(); for(;;); }

#endif /* FREERTOS_CONFIG_H */
