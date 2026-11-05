#pragma once
// Kiểm thử tuần tự các thứ tự ISR/task; không mô phỏng bộ lập lịch FreeRTOS.
using portMUX_TYPE = int;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(khoa) ((void)(khoa))
#define portEXIT_CRITICAL(khoa) ((void)(khoa))
#define portENTER_CRITICAL_ISR(khoa) ((void)(khoa))
#define portEXIT_CRITICAL_ISR(khoa) ((void)(khoa))
