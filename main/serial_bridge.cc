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

static inline int u64_to_dec(uint64_t v, char* out, size_t out_sz) {
    char buf[21]; // max for 64-bit
    size_t i = 0;
    if (v == 0) { if (out_sz > 1) { out[0] = '0'; out[1] = '\0'; return 1; } return 0; }
    while (v && i < sizeof(buf)) { buf[i++] = (char)('0' + (v % 10)); v /= 10; }
    size_t o = 0;
    while (i && o + 1 < out_sz) { out[o++] = buf[--i]; }
    out[o] = '\0';
    return (int)o;
}

static inline void append_str_bounded(char* dst, size_t dst_sz, size_t& o, const char* s) {
    if (!s) s = "";
    while (*s && o + 1 < dst_sz) { dst[o++] = *s++; }
    dst[o] = '\0';
}

int SerialBridge::write_line_(const char* tag, const char* type, const char* msg) {
    if (!enabled_) return 0;
    if (!tag) tag = "";
    if (!type) type = "";
    if (!msg) msg = "";

    // Prepare escaped message
    char msg_esc[512];
    escape_json_(msg, msg_esc, sizeof(msg_esc));

    uint64_t ts_ms = (uint64_t)(esp_timer_get_time() / 1000ULL);
    char ts_buf[24];
    u64_to_dec(ts_ms, ts_buf, sizeof(ts_buf));

    // Build JSON line without printf to avoid varargs pitfalls
    char line[768];
    size_t o = 0;
    append_str_bounded(line, sizeof(line), o, "{\"ts\":");
    append_str_bounded(line, sizeof(line), o, ts_buf);
    append_str_bounded(line, sizeof(line), o, ",\"tag\":\"");
    append_str_bounded(line, sizeof(line), o, tag);
    append_str_bounded(line, sizeof(line), o, "\",\"type\":\"");
    append_str_bounded(line, sizeof(line), o, type);
    append_str_bounded(line, sizeof(line), o, "\",\"msg\":\"");
    append_str_bounded(line, sizeof(line), o, msg_esc);
    append_str_bounded(line, sizeof(line), o, "\"}\n");

    if (mutex_) xSemaphoreTake(mutex_, portMAX_DELAY);
    int wrote = uart_write_bytes(uart_num_, line, (size_t)o);
    if (mutex_) xSemaphoreGive(mutex_);
    return wrote;
}

void SerialBridge::Sendf(const char* tag, const char* type, const char* fmt, ...) {
    if (!enabled_) return;
    if (!fmt) return;
    char msg[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);
    write_line_(tag, type, msg);
}

