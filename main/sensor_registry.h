#ifndef SENSOR_REGISTRY_H
#define SENSOR_REGISTRY_H

#include <string>
#include <cstdint>
#include <cJSON.h>

// Thread-safe registry for most-recent sensor values reported from the MCU
// via UART JSON lines. Keys are dynamic and determined by incoming JSON.
//
// Example MCU payload (one JSON line ending with \n):
//   {"tag":"MCU","type":"sensor_update","data":{"temp":23.6,"hum":46.2},"ts":1736450001}
//
// Call SensorRegistry::UpdateFromJson(data) with the "data" object to update values.
class SensorRegistry {
public:
    // Merge keys from a JSON object into the registry.
    // Numbers are stored as numeric; strings as string; objects/arrays are serialized to string.
    static void UpdateFromJson(const cJSON* obj);

    // Get a numeric value by key. Returns true if found. age_ms returns how old the value is.
    static bool GetDouble(const std::string& key, double& out_value, uint64_t& age_ms);

    // Get a string value by key. Returns true if found. age_ms returns how old the value is.
    static bool GetString(const std::string& key, std::string& out_value, uint64_t& age_ms);

    // Dump current registry as a cJSON object (caller owns and must cJSON_Delete).
    static cJSON* DumpJson();
};

#endif // SENSOR_REGISTRY_H

