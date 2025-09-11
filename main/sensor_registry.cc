#include "sensor_registry.h"

#include <map>
#include <mutex>
#include <cstring>
#include <esp_timer.h>

namespace {
struct Entry {
    bool is_number = false;
    double number = 0.0;
    std::string text;
    uint64_t ts_ms = 0;
};

std::mutex g_mutex;
std::map<std::string, Entry> g_store;

static inline uint64_t now_ms() {
    return (uint64_t)(esp_timer_get_time() / 1000ULL);
}

static void set_entry_number(const std::string& key, double v) {
    std::lock_guard<std::mutex> lock(g_mutex);
    auto& e = g_store[key];
    e.is_number = true;
    e.number = v;
    e.text.clear();
    e.ts_ms = now_ms();
}

static void set_entry_string(const std::string& key, const std::string& v) {
    std::lock_guard<std::mutex> lock(g_mutex);
    auto& e = g_store[key];
    e.is_number = false;
    e.text = v;
    e.number = 0.0;
    e.ts_ms = now_ms();
}
}

void SensorRegistry::UpdateFromJson(const cJSON* obj) {
    if (!obj || !cJSON_IsObject(obj)) return;

    for (const cJSON* it = obj->child; it != nullptr; it = it->next) {
        if (!it->string) continue;
        const std::string key(it->string);
        if (cJSON_IsNumber(it)) {
            set_entry_number(key, it->valuedouble);
        } else if (cJSON_IsString(it)) {
            set_entry_string(key, it->valuestring ? it->valuestring : "");
        } else {
            // For objects/arrays/bools: serialize to compact string
            char* s = cJSON_PrintUnformatted((cJSON*)it);
            if (s) {
                set_entry_string(key, std::string(s));
                cJSON_free(s);
            }
        }
    }
}

bool SensorRegistry::GetDouble(const std::string& key, double& out_value, uint64_t& age_ms) {
    std::lock_guard<std::mutex> lock(g_mutex);
    auto it = g_store.find(key);
    if (it == g_store.end()) return false;
    if (!it->second.is_number) return false;
    out_value = it->second.number;
    age_ms = (it->second.ts_ms > 0) ? (now_ms() - it->second.ts_ms) : UINT64_C(0);
    return true;
}

bool SensorRegistry::GetString(const std::string& key, std::string& out_value, uint64_t& age_ms) {
    std::lock_guard<std::mutex> lock(g_mutex);
    auto it = g_store.find(key);
    if (it == g_store.end()) return false;
    if (it->second.is_number) return false;
    out_value = it->second.text;
    age_ms = (it->second.ts_ms > 0) ? (now_ms() - it->second.ts_ms) : UINT64_C(0);
    return true;
}

cJSON* SensorRegistry::DumpJson() {
    std::lock_guard<std::mutex> lock(g_mutex);
    cJSON* root = cJSON_CreateObject();
    for (const auto& kv : g_store) {
        if (kv.second.is_number) {
            cJSON_AddNumberToObject(root, kv.first.c_str(), kv.second.number);
        } else {
            cJSON_AddStringToObject(root, kv.first.c_str(), kv.second.text.c_str());
        }
    }
    return root;
}

