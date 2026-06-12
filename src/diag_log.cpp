#include "obd_data.h"
#include <stdarg.h>

DiagEntry g_diag_log[DIAG_LOG_SIZE] = {};
volatile uint16_t g_diag_head = 0;
SemaphoreHandle_t g_diag_mutex = nullptr;

void diag_log_init() {
    g_diag_mutex = xSemaphoreCreateMutex();
}

void diag_log(const char* fmt, ...) {
    if (!g_diag_mutex) return;
    char tmp[DIAG_MSG_LEN];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(tmp, sizeof(tmp), fmt, ap);
    va_end(ap);

    // Also mirror to serial
    Serial.println(tmp);

    if (xSemaphoreTake(g_diag_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        uint8_t idx = g_diag_head % DIAG_LOG_SIZE;
        g_diag_log[idx].ts_ms = millis();
        strncpy(g_diag_log[idx].msg, tmp, DIAG_MSG_LEN - 1);
        g_diag_log[idx].msg[DIAG_MSG_LEN - 1] = '\0';
        g_diag_head++;
        xSemaphoreGive(g_diag_mutex);
    }
}
