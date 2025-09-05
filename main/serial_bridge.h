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

private:
    static bool enabled_;
    static uart_port_t uart_num_;
    static SemaphoreHandle_t mutex_;

    static int write_line_(const char* tag, const char* type, const char* msg);
    static int escape_json_(const char* in, char* out, size_t out_sz);
};

#endif // SERIAL_BRIDGE_H

