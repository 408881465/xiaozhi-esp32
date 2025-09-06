#include "serial_bridge.h"
#include "sdkconfig.h"

#include <cstring>
#include <cstdio>
#include <esp_timer.h>
#include <driver/gpio.h>

bool SerialBridge::enabled_ = false;
uart_port_t SerialBridge::uart_num_ = UART_NUM_1;
SemaphoreHandle_t SerialBridge::mutex_ = nullptr;

static unsigned int s_event_id = 1;

#ifdef CONFIG_SERIAL_BRIDGE_EMIT_PLAN
static bool s_emit_plan = true;
#else
static bool s_emit_plan = false;
#endif

void SerialBridge::SetEmitPlan(bool enable) {
    s_emit_plan = enable;
}

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

    // Install driver: 2KB TX buffer; enable RX buffer only when RX pin is provided
    int rx_buf = (rx_pin >= 0) ? 256 : 0;
    uart_driver_install(uart_num_, 2048, rx_buf, 0, nullptr, 0);
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



static inline int i32_to_dec(int v, char* out, size_t out_sz) {
    if (out_sz == 0) return 0;
    if (v == 0) { if (out_sz > 1) { out[0] = '0'; out[1] = '\0'; return 1; } return 0; }
    bool neg = v < 0; unsigned int uv = neg ? (unsigned int)(-v) : (unsigned int)v;
    char buf[12]; size_t i = 0;
    while (uv && i < sizeof(buf)) { buf[i++] = (char)('0' + (uv % 10)); uv /= 10; }
    size_t o = 0;
    if (neg && o + 1 < out_sz) out[o++] = '-';
    while (i && o + 1 < out_sz) { out[o++] = buf[--i]; }
    out[o] = '\0';
    return (int)o;
}

void SerialBridge::SendMcpToolCall(const char* device, const char* action) {
    if (!enabled_) return;
    if (!device) {
        device = "";
    }
    if (!action) {
        action = "";
    }
    char dev_esc[128]; char act_esc[128];
    escape_json_(device, dev_esc, sizeof(dev_esc));
    escape_json_(action, act_esc, sizeof(act_esc));

    uint64_t ts_ms = (uint64_t)(esp_timer_get_time() / 1000ULL);
    char ts_buf[24]; u64_to_dec(ts_ms, ts_buf, sizeof(ts_buf));

    char line[256]; size_t o = 0;
    append_str_bounded(line, sizeof(line), o, "{\"ts\":");
    append_str_bounded(line, sizeof(line), o, ts_buf);
    append_str_bounded(line, sizeof(line), o, ",\"tag\":\"MCP\",\"type\":\"tool_call\",");
    append_str_bounded(line, sizeof(line), o, "\"device\":\"");
    append_str_bounded(line, sizeof(line), o, dev_esc);
    append_str_bounded(line, sizeof(line), o, "\",\"action\":\"");
    append_str_bounded(line, sizeof(line), o, act_esc);
    append_str_bounded(line, sizeof(line), o, "\"}\n");

    if (mutex_) xSemaphoreTake(mutex_, portMAX_DELAY);
    uart_write_bytes(uart_num_, line, o);
    if (mutex_) xSemaphoreGive(mutex_);
}

static void append_num_field(char* line, size_t& o, size_t cap, const char* key, int v, bool first) {
    if (!first) append_str_bounded(line, cap, o, ",");
    append_str_bounded(line, cap, o, "\"");
    append_str_bounded(line, cap, o, key);
    append_str_bounded(line, cap, o, "\":");
    char nbuf[16]; i32_to_dec(v, nbuf, sizeof(nbuf));
    append_str_bounded(line, cap, o, nbuf);
}

static void begin_common(char* line, size_t& o, size_t cap, const char* tag, const char* type) {
    uint64_t ts_ms = (uint64_t)(esp_timer_get_time() / 1000ULL);
    char ts_buf[24]; u64_to_dec(ts_ms, ts_buf, sizeof(ts_buf));
    append_str_bounded(line, cap, o, "{\"ts\":");
    append_str_bounded(line, cap, o, ts_buf);
    append_str_bounded(line, cap, o, ",\"tag\":\"");
    append_str_bounded(line, cap, o, tag);
    append_str_bounded(line, cap, o, "\",\"type\":\"");
    append_str_bounded(line, cap, o, type);
    append_str_bounded(line, cap, o, "\"");
}

static void end_line(char* line, size_t& o, size_t cap) {
    append_str_bounded(line, cap, o, "}\n");
}

void SerialBridge::SendLedSetBrightness(int level, int parent_id) {
    if (!enabled_) return;
    char line[224]; size_t o = 0; begin_common(line, o, sizeof(line), "LedStrip", "set_brightness");
    if (parent_id >= 0) {
        append_str_bounded(line, sizeof(line), o, ",\"parent_id\":");
        char pid[16]; i32_to_dec(parent_id, pid, sizeof(pid)); append_str_bounded(line, sizeof(line), o, pid);
    }
    append_str_bounded(line, sizeof(line), o, ",\"level\":");
    char nbuf[16]; i32_to_dec(level, nbuf, sizeof(nbuf));
    append_str_bounded(line, sizeof(line), o, nbuf);
    end_line(line, o, sizeof(line));
    if (mutex_) xSemaphoreTake(mutex_, portMAX_DELAY);
    uart_write_bytes(uart_num_, line, o);
    if (mutex_) xSemaphoreGive(mutex_);
}

void SerialBridge::SendLedSetSingleColor(int idx, int r, int g, int b, int parent_id) {
    if (!enabled_) return;
    char line[256]; size_t o = 0; begin_common(line, o, sizeof(line), "LedStrip", "set_single_color");
    if (parent_id >= 0) {
        append_str_bounded(line, sizeof(line), o, ",\"parent_id\":");
        char pid[16]; i32_to_dec(parent_id, pid, sizeof(pid)); append_str_bounded(line, sizeof(line), o, pid);
    }
    append_num_field(line, o, sizeof(line), "idx", idx, false);
    append_num_field(line, o, sizeof(line), "r", r, false);
    append_num_field(line, o, sizeof(line), "g", g, false);
    append_num_field(line, o, sizeof(line), "b", b, false);
    end_line(line, o, sizeof(line));
    if (mutex_) xSemaphoreTake(mutex_, portMAX_DELAY);
    uart_write_bytes(uart_num_, line, o);
    if (mutex_) xSemaphoreGive(mutex_);
}

void SerialBridge::SendLedSetAllColor(int r, int g, int b, int parent_id) {
    if (!enabled_) return;
    char line[224]; size_t o = 0; begin_common(line, o, sizeof(line), "LedStrip", "set_all_color");
    if (parent_id >= 0) {
        append_str_bounded(line, sizeof(line), o, ",\"parent_id\":");
        char pid[16]; i32_to_dec(parent_id, pid, sizeof(pid)); append_str_bounded(line, sizeof(line), o, pid);
    }
    append_num_field(line, o, sizeof(line), "r", r, false);
    append_num_field(line, o, sizeof(line), "g", g, false);
    append_num_field(line, o, sizeof(line), "b", b, false);
    end_line(line, o, sizeof(line));
    if (mutex_) xSemaphoreTake(mutex_, portMAX_DELAY);
    uart_write_bytes(uart_num_, line, o);
    if (mutex_) xSemaphoreGive(mutex_);
}

void SerialBridge::SendLedBlink(int r, int g, int b, int interval_ms, int parent_id) {
    if (!enabled_) return;
    char line[256]; size_t o = 0; begin_common(line, o, sizeof(line), "LedStrip", "blink");
    if (parent_id >= 0) {
        append_str_bounded(line, sizeof(line), o, ",\"parent_id\":");
        char pid[16]; i32_to_dec(parent_id, pid, sizeof(pid)); append_str_bounded(line, sizeof(line), o, pid);
    }
    append_num_field(line, o, sizeof(line), "r", r, false);
    append_num_field(line, o, sizeof(line), "g", g, false);
    append_num_field(line, o, sizeof(line), "b", b, false);
    append_num_field(line, o, sizeof(line), "interval", interval_ms, false);
    end_line(line, o, sizeof(line));
    if (mutex_) xSemaphoreTake(mutex_, portMAX_DELAY);
    uart_write_bytes(uart_num_, line, o);
    if (mutex_) xSemaphoreGive(mutex_);
}

void SerialBridge::SendLedScroll(int r, int g, int b, int length, int interval_ms, int parent_id) {
    if (!enabled_) return;
    char line[272]; size_t o = 0; begin_common(line, o, sizeof(line), "LedStrip", "scroll");
    if (parent_id >= 0) {
        append_str_bounded(line, sizeof(line), o, ",\"parent_id\":");
        char pid[16]; i32_to_dec(parent_id, pid, sizeof(pid)); append_str_bounded(line, sizeof(line), o, pid);
    }
    append_num_field(line, o, sizeof(line), "r", r, false);
    append_num_field(line, o, sizeof(line), "g", g, false);
    append_num_field(line, o, sizeof(line), "b", b, false);
    append_num_field(line, o, sizeof(line), "length", length, false);
    append_num_field(line, o, sizeof(line), "interval", interval_ms, false);
    end_line(line, o, sizeof(line));
    if (mutex_) xSemaphoreTake(mutex_, portMAX_DELAY);
    uart_write_bytes(uart_num_, line, o);
    if (mutex_) xSemaphoreGive(mutex_);
}


static inline const char* skip_ws_(const char* p){ while(p && (*p==' '||*p=='\t')) ++p; return p; }
static inline bool is_digit_str_(const char* s){ if(!s||!*s) return false; if(*s=='-'&&*(s+1)) ++s; for(const char* q=s; *q; ++q){ if(*q<'0'||*q>'9') return false; } return true; }

void SerialBridge::SendMcpToolCallWithParams(const char* device, const char* action, const char* params_kv) {
    if (!enabled_) return;
    if (!device) {
        device = "";
    }
    if (!action) {
        action = "";
    }
    char dev_esc[128]; char act_esc[128];
    escape_json_(device, dev_esc, sizeof(dev_esc));
    escape_json_(action, act_esc, sizeof(act_esc));

    uint64_t ts_ms = (uint64_t)(esp_timer_get_time() / 1000ULL);
    char ts_buf[24]; u64_to_dec(ts_ms, ts_buf, sizeof(ts_buf));

    char line[512]; size_t o = 0;
    append_str_bounded(line, sizeof(line), o, "{\"ts\":");
    append_str_bounded(line, sizeof(line), o, ts_buf);
    append_str_bounded(line, sizeof(line), o, ",\"tag\":\"MCP\",\"type\":\"tool_call\",");
    append_str_bounded(line, sizeof(line), o, "\"device\":\"");
    append_str_bounded(line, sizeof(line), o, dev_esc);
    append_str_bounded(line, sizeof(line), o, "\",\"action\":\"");
    append_str_bounded(line, sizeof(line), o, act_esc);
    append_str_bounded(line, sizeof(line), o, "\",\"params\":{");

    // Parse params_kv like: "r=255,g=0,b=0" into JSON object
    bool first = true;
    if (params_kv) {
        const char* p = params_kv;
        while (p && *p) {
            p = skip_ws_(p);
            // parse key
            char key[48] = {0}; size_t ki = 0;
            while (*p && *p!='=' && *p!=',' && *p!=')' && *p!=' ' && *p!='\t' && ki < sizeof(key)-1) key[ki++] = *p++;
            key[ki] = '\0';
            while (*p==' '||*p=='\t') ++p;
            if (*p != '=') { // skip to next comma
                while (*p && *p != ',') {
                    ++p;
                }
                if (*p == ',') {
                    ++p;
                }
                continue;
            }
            ++p; // skip '='
            p = skip_ws_(p);

            // parse value token
            char val_raw[80] = {0}; size_t vi = 0;
            while (*p && *p!=',' && *p!=')' && vi < sizeof(val_raw)-1) {
                // stop at trailing spaces before comma/)
                if (*p==' '||*p=='\t') { const char* q = p; while(*q==' '||*q=='\t') ++q; if (*q==','||*q==')'||*q=='\0'){ p = q; break; } }
                val_raw[vi++] = *p++;
            }
            val_raw[vi] = '\0';

            if (key[0]) {
                if (!first) append_str_bounded(line, sizeof(line), o, ",");
                first = false;
                // key
                append_str_bounded(line, sizeof(line), o, "\"");
                char key_esc[96]; escape_json_(key, key_esc, sizeof(key_esc));
                append_str_bounded(line, sizeof(line), o, key_esc);
                append_str_bounded(line, sizeof(line), o, "\":");
                // value
                if (is_digit_str_(val_raw)) {
                    append_str_bounded(line, sizeof(line), o, val_raw);
                } else {
                    append_str_bounded(line, sizeof(line), o, "\"");
                    char val_esc[160]; escape_json_(val_raw, val_esc, sizeof(val_esc));
                    append_str_bounded(line, sizeof(line), o, val_esc);
                    append_str_bounded(line, sizeof(line), o, "\"");
                }
            }
            if (*p==',') ++p; // move to next pair
        }
    }

    append_str_bounded(line, sizeof(line), o, "}}\n");

    if (mutex_) xSemaphoreTake(mutex_, portMAX_DELAY);
    uart_write_bytes(uart_num_, line, o);
    if (mutex_) xSemaphoreGive(mutex_);
}

unsigned int SerialBridge::SendMcpPlanWithParams(const char* device, const char* action, const char* params_kv) {
    if (!enabled_) return 0;
    if (!s_emit_plan) return 0;
    if (!device) { device = ""; }
    if (!action) { action = ""; }

    // Prepare escaped strings
    char dev_esc[128]; char act_esc[128];
    escape_json_(device, dev_esc, sizeof(dev_esc));
    escape_json_(action, act_esc, sizeof(act_esc));

    // Build line
    if (mutex_) xSemaphoreTake(mutex_, portMAX_DELAY);
    unsigned int id = s_event_id++;
    uint64_t ts_ms = (uint64_t)(esp_timer_get_time() / 1000ULL);
    char ts_buf[24]; u64_to_dec(ts_ms, ts_buf, sizeof(ts_buf));
    char id_buf[16]; i32_to_dec((int)id, id_buf, sizeof(id_buf));

    char line[560]; size_t o = 0;
    append_str_bounded(line, sizeof(line), o, "{\"ts\":");
    append_str_bounded(line, sizeof(line), o, ts_buf);
    append_str_bounded(line, sizeof(line), o, ",\"id\":");
    append_str_bounded(line, sizeof(line), o, id_buf);
    append_str_bounded(line, sizeof(line), o, ",\"tag\":\"MCP\",\"type\":\"tool_call\",\"stage\":\"plan\",");
    append_str_bounded(line, sizeof(line), o, "\"device\":\"");
    append_str_bounded(line, sizeof(line), o, dev_esc);
    append_str_bounded(line, sizeof(line), o, "\",\"action\":\"");
    append_str_bounded(line, sizeof(line), o, act_esc);
    append_str_bounded(line, sizeof(line), o, "\",\"params\":{");

    // Parse params if any (plan often has none)
    bool first = true;
    if (params_kv) {
        const char* p = params_kv;
        while (p && *p) {
            p = skip_ws_(p);
            char key[48] = {0}; size_t ki = 0;
            while (*p && *p!='=' && *p!=',' && *p!=')' && *p!=' ' && *p!='\t' && ki < sizeof(key)-1) key[ki++] = *p++;
            key[ki] = '\0';
            while (*p==' '||*p=='\t') ++p;
            if (*p != '=') { while (*p && *p != ',') { ++p; } if (*p == ',') { ++p; } continue; }
            ++p; p = skip_ws_(p);
            char val_raw[80] = {0}; size_t vi = 0;
            while (*p && *p!=',' && *p!=')' && vi < sizeof(val_raw)-1) {
                if (*p==' '||*p=='\t') { const char* q=p; while(*q==' '||*q=='\t') ++q; if (*q==','||*q==')'||*q=='\0'){ p=q; break; } }
                val_raw[vi++] = *p++;
            }
            val_raw[vi] = '\0';
            if (key[0]) {
                if (!first) append_str_bounded(line, sizeof(line), o, ",");
                first = false;
                append_str_bounded(line, sizeof(line), o, "\"");
                char key_esc[96]; escape_json_(key, key_esc, sizeof(key_esc));
                append_str_bounded(line, sizeof(line), o, key_esc);
                append_str_bounded(line, sizeof(line), o, "\":");
                if (is_digit_str_(val_raw)) {
                    append_str_bounded(line, sizeof(line), o, val_raw);
                } else {
                    append_str_bounded(line, sizeof(line), o, "\"");
                    char val_esc[160]; escape_json_(val_raw, val_esc, sizeof(val_esc));
                    append_str_bounded(line, sizeof(line), o, val_esc);
                    append_str_bounded(line, sizeof(line), o, "\"");
                }
            }
            if (*p==',') ++p;
        }
    }
    append_str_bounded(line, sizeof(line), o, "}}\n");

    uart_write_bytes(uart_num_, line, o);
    if (mutex_) xSemaphoreGive(mutex_);
    return id;
}

unsigned int SerialBridge::SendMcpExecWithParams(const char* device, const char* action, const char* params_kv) {
    if (!enabled_) return 0;
    if (!device) { device = ""; }
    if (!action) { action = ""; }

    char dev_esc[128]; char act_esc[128];
    escape_json_(device, dev_esc, sizeof(dev_esc));
    escape_json_(action, act_esc, sizeof(act_esc));

    if (mutex_) xSemaphoreTake(mutex_, portMAX_DELAY);
    unsigned int id = s_event_id++;
    uint64_t ts_ms = (uint64_t)(esp_timer_get_time() / 1000ULL);
    char ts_buf[24]; u64_to_dec(ts_ms, ts_buf, sizeof(ts_buf));
    char id_buf[16]; i32_to_dec((int)id, id_buf, sizeof(id_buf));

    char line[560]; size_t o = 0;
    append_str_bounded(line, sizeof(line), o, "{\"ts\":");
    append_str_bounded(line, sizeof(line), o, ts_buf);
    append_str_bounded(line, sizeof(line), o, ",\"id\":");
    append_str_bounded(line, sizeof(line), o, id_buf);
    append_str_bounded(line, sizeof(line), o, ",\"tag\":\"MCP\",\"type\":\"tool_call\",\"stage\":\"exec\",");
    append_str_bounded(line, sizeof(line), o, "\"device\":\"");
    append_str_bounded(line, sizeof(line), o, dev_esc);
    append_str_bounded(line, sizeof(line), o, "\",\"action\":\"");
    append_str_bounded(line, sizeof(line), o, act_esc);
    append_str_bounded(line, sizeof(line), o, "\",\"params\":{");

    bool first = true;
    if (params_kv) {
        const char* p = params_kv;
        while (p && *p) {
            p = skip_ws_(p);
            char key[48] = {0}; size_t ki = 0;
            while (*p && *p!='=' && *p!=',' && *p!=')' && *p!=' ' && *p!='\t' && ki < sizeof(key)-1) key[ki++] = *p++;
            key[ki] = '\0';
            while (*p==' '||*p=='\t') ++p;
            if (*p != '=') { while (*p && *p != ',') { ++p; } if (*p == ',') { ++p; } continue; }
            ++p; p = skip_ws_(p);
            char val_raw[80] = {0}; size_t vi = 0;
            while (*p && *p!=',' && *p!=')' && vi < sizeof(val_raw)-1) {
                if (*p==' '||*p=='\t') { const char* q=p; while(*q==' '||*q=='\t') ++q; if (*q==','||*q==')'||*q=='\0'){ p=q; break; } }
                val_raw[vi++] = *p++;
            }
            val_raw[vi] = '\0';
            if (key[0]) {
                if (!first) append_str_bounded(line, sizeof(line), o, ",");
                first = false;
                append_str_bounded(line, sizeof(line), o, "\"");
                char key_esc[96]; escape_json_(key, key_esc, sizeof(key_esc));
                append_str_bounded(line, sizeof(line), o, key_esc);
                append_str_bounded(line, sizeof(line), o, "\":");
                if (is_digit_str_(val_raw)) {
                    append_str_bounded(line, sizeof(line), o, val_raw);
                } else {
                    append_str_bounded(line, sizeof(line), o, "\"");
                    char val_esc[160]; escape_json_(val_raw, val_esc, sizeof(val_esc));
                    append_str_bounded(line, sizeof(line), o, val_esc);
                    append_str_bounded(line, sizeof(line), o, "\"");
                }
            }
            if (*p==',') ++p;
        }
    }
    append_str_bounded(line, sizeof(line), o, "}}\n");

    uart_write_bytes(uart_num_, line, o);
    if (mutex_) xSemaphoreGive(mutex_);
    return id;
}



void SerialBridge::SendAppGarbageSort(const char* category, int /*category_code*/, const char* /*id*/, const char* /*source*/, const char* /*ver*/, const char* /*category_zh*/) {
    if (!enabled_) return;
    if (!category) category = "";

    char cat_esc[64];
    escape_json_(category, cat_esc, sizeof(cat_esc));

    char line[192]; size_t o = 0;
    begin_common(line, o, sizeof(line), "Application", "<<");
    append_str_bounded(line, sizeof(line), o, ",\"topic\":\"garbage_sort\"");
    append_str_bounded(line, sizeof(line), o, ",\"category\":\"");
    append_str_bounded(line, sizeof(line), o, cat_esc);
    append_str_bounded(line, sizeof(line), o, "\"");
    end_line(line, o, sizeof(line));

    if (mutex_) xSemaphoreTake(mutex_, portMAX_DELAY);
    uart_write_bytes(uart_num_, line, o);
    if (mutex_) xSemaphoreGive(mutex_);
}


void SerialBridge::SendAppMsgWithGarbage(const char* msg, const char* category) {
    if (!enabled_) return;
    if (!msg) msg = "";
    if (!category) category = "";

    char msg_esc[256];
    char cat_esc[64];
    escape_json_(msg, msg_esc, sizeof(msg_esc));
    escape_json_(category, cat_esc, sizeof(cat_esc));

    char line[512]; size_t o = 0;
    begin_common(line, o, sizeof(line), "Application", "<<");
    append_str_bounded(line, sizeof(line), o, ",\"msg\":\"");
    append_str_bounded(line, sizeof(line), o, msg_esc);
    append_str_bounded(line, sizeof(line), o, "\"");
    append_str_bounded(line, sizeof(line), o, ",\"topic\":\"garbage_sort\"");
    append_str_bounded(line, sizeof(line), o, ",\"category\":\"");
    append_str_bounded(line, sizeof(line), o, cat_esc);
    append_str_bounded(line, sizeof(line), o, "\"" );
    end_line(line, o, sizeof(line));

    if (mutex_) xSemaphoreTake(mutex_, portMAX_DELAY);
    uart_write_bytes(uart_num_, line, o);
    if (mutex_) xSemaphoreGive(mutex_);
}
