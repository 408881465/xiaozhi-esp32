#ifndef SERIAL_BRIDGE_H
#define SERIAL_BRIDGE_H

#include <cstdarg>
#include <cstddef>
#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <driver/uart.h>

// A very small UART-based event bridge to a downstream MCU.
// Usage:
//   SerialBridge::Initialize(UART_NUM_1, tx_pin, rx_pin_or_-1, 115200);
//   SerialBridge::Sendf("Application", "state", "%s", "speaking");
//
// If Initialize() is never called, Sendf() is a no-op.
class SerialBridge {
public:
    static void Initialize(uart_port_t uart_num, int tx_pin, int rx_pin, int baudrate);
    static void Sendf(const char* tag, const char* type, const char* fmt, ...);

    // Structured JSON helpers (field-based for easy parsing)
    static void SendMcpToolCall(const char* device, const char* action);
    static void SendMcpToolCallWithParams(const char* device, const char* action, const char* params_kv);
    static unsigned int SendMcpPlanWithParams(const char* device, const char* action, const char* params_kv);
    static unsigned int SendMcpExecWithParams(const char* device, const char* action, const char* params_kv);

    // Application specific helpers
    // 1) Emit a structured line for garbage classification (legacy helper; now minimized fields)
    static void SendAppGarbageSort(const char* category, int category_code, const char* id = nullptr, const char* source = "cloud", const char* ver = "1.0", const char* category_zh = nullptr);
    // 2) Emit a single Application line that contains both msg and garbage_sort fields
    static void SendAppMsgWithGarbage(const char* msg, const char* category);

    // Runtime toggle: emit or suppress plan events
    static void SetEmitPlan(bool enable);

    static void SendLedSetBrightness(int level, int parent_id = -1);
    static void SendLedSetSingleColor(int idx, int r, int g, int b, int parent_id = -1);
    static void SendLedSetAllColor(int r, int g, int b, int parent_id = -1);
    static void SendLedBlink(int r, int g, int b, int interval_ms, int parent_id = -1);
    static void SendLedScroll(int r, int g, int b, int length, int interval_ms, int parent_id = -1);

private:
    static bool enabled_;
    static uart_port_t uart_num_;
    static SemaphoreHandle_t mutex_;

    // RX task: read UART lines and parse MCU -> ESP32 sensor updates
    static void rx_task_(void*);


    static int write_line_(const char* tag, const char* type, const char* msg);
    static int escape_json_(const char* in, char* out, size_t out_sz);
};

#endif // SERIAL_BRIDGE_H

