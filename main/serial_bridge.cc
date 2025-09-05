#include "serial_bridge.h"

#include <cstring>
#include <cstdio>
#include <esp_timer.h>
#include <driver/gpio.h>

bool SerialBridge::enabled_ = false;
uart_port_t SerialBridge::uart_num_ = UART_NUM_1;
SemaphoreHandle_t SerialBridge::mutex_ = nullptr;

void SerialBridge::Initialize(uart_port_t uart_num, int tx_pin, int rx_pin, int baudrate) {
    if (enabled_) return;

    uart_config_t uart_config = {};
    uart_config.baud_rate = baudrate;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_APB;

    uart_num_ = uart_num;
    // Install driver (TX buffer 2KB, no RX)
    uart_driver_install(uart_num_, 2048, 0, 0, nullptr, 0);
    uart_param_config(uart_num_, &uart_config);

    int rx = (rx_pin >= 0) ? rx_pin : UART_PIN_NO_CHANGE;
    uart_set_pin(uart_num_, tx_pin, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (rx_pin >= 0) {
        // Ensure RX idles high when cable is not connected
        gpio_set_pull_mode((gpio_num_t)rx_pin, GPIO_PULLUP_ONLY);
        uart_flush_input(uart_num_);
    }

    if (!mutex_) mutex_ = xSemaphoreCreateMutex();
    enabled_ = true;
}

int SerialBridge::escape_json_(const char* in, char* out, size_t out_sz) {
    size_t o = 0;
    for (size_t i = 0; in[i] != '\0'; ++i) {
        char c = in[i];
        const char* esc = nullptr;
        char small[3] = {0};
        switch (c) {
            case '"': esc = "\\\""; break;
            case '\\': esc = "\\\\"; break;
            case '\n': esc = "\\n"; break;
            case '\r': esc = "\\r"; break;
            case '\t': esc = "\\t"; break;
            default:
                small[0] = c; small[1] = '\0'; esc = small; break;
        }
        size_t elen = strlen(esc);
        if (o + elen + 1 >= out_sz) break;
        memcpy(out + o, esc, elen);
        o += elen;
    }
    out[o] = '\0';
    return (int)o;
}

int SerialBridge::write_line_(const char* tag, const char* type, const char* msg) {
    if (!enabled_) return 0;

    // Prepare JSON line
    char msg_esc[512];
    escape_json_(msg, msg_esc, sizeof(msg_esc));

    uint64_t ts_ms = (uint64_t)(esp_timer_get_time() / 1000ULL);
    char line[768];
    int n = snprintf(line, sizeof(line),
                     "{\"ts\":%llu,\"tag\":\"%s\",\"type\":\"%s\",\"msg\":\"%s\"}\n",
                     (unsigned long long)ts_ms, tag, type, msg_esc);
    if (n < 0) return n;

    if (mutex_) xSemaphoreTake(mutex_, portMAX_DELAY);
    int wrote = uart_write_bytes(uart_num_, line, n);
    if (mutex_) xSemaphoreGive(mutex_);
    return wrote;
}

void SerialBridge::Sendf(const char* tag, const char* type, const char* fmt, ...) {
    if (!enabled_) return;
    char msg[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);
    write_line_(tag, type, msg);
}

