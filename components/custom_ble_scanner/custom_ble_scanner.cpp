// custom_components/custom_ble_scanner/custom_ble_scanner.cpp
#include "custom_ble_scanner.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h" // Includes esphome::format_hex_pretty
#include "esphome/core/application.h" // Needed for App.get_name()
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/mqtt/mqtt_client.h"
#include "esphome/components/json/json_util.h"
#include <cmath>     // Added for round() function
#include <algorithm> // Added for std::transform
#include <cctype>    // Added for ::tolower

namespace esphome {
namespace custom_ble_scanner {

static const char *TAG = "custom_ble_scanner";

// Helper function to convert string to lowercase
std::string to_lower_string(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c){ return std::tolower(c); });
  return s;
}

// Helper function to convert MAC address (uint64_t) to string format AA:BB:CC:DD:EE:FF
std::string mac_address_to_string(uint64_t mac) {
  char buffer[18];
  snprintf(buffer, sizeof(buffer), "%02X:%02X:%02X:%02X:%02X:%02X",
           (uint8_t)(mac >> 40), (uint8_t)(mac >> 32), (uint8_t)(mac >> 24),
           (uint8_t)(mac >> 16), (uint8_t)(mac >> 8), (uint8_t)(mac >> 0));
  return std::string(buffer);
}

// Overload: Helper function to convert MAC address (uint8_t array) to string format AA:BB:CC:DD:EE:FF
std::string mac_address_to_string(const uint8_t* mac_array) {
  char buffer[18];
  snprintf(buffer, sizeof(buffer), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac_array[0], mac_array[1], mac_array[2],
           mac_array[3], mac_array[4], mac_array[5]);
  return std::string(buffer);
}


// Helper function to convert MAC address (uint64_t) to clean string format aa_bb_cc_dd_ee_ff for MQTT topics
std::string mac_address_to_clean_string(uint64_t mac) {
  char buffer[18];
  snprintf(buffer, sizeof(buffer), "%02x_%02x_%02x_%02x_%02x_%02x", // Use %02x for lowercase hex
           (uint8_t)(mac >> 40), (uint8_t)(mac >> 32), (uint8_t)(mac >> 24),
           (uint8_t)(mac >> 16), (uint8_t)(mac >> 8), (uint8_t)(mac >> 0));
  return std::string(buffer);
}

// Overload: Helper function to convert MAC address (uint8_t array) to clean string format aa_bb_cc_dd_ee_ff for MQTT topics
std::string mac_address_to_clean_string(const uint8_t* mac_array) {
  char buffer[18];
  snprintf(buffer, sizeof(buffer), "%02x_%02x_%02x_%02x_%02x_%02x", // Use %02x for lowercase hex
           mac_array[0], mac_array[1], mac_array[2],
           mac_array[3], mac_array[4], mac_array[5]);
  return std::string(buffer);
}

std::string format_vector_to_hex(const std::vector<uint8_t>& data) {
  if (data.empty()) {
    return "";
  }
  return esphome::format_hex_pretty(data);
}

// --- CustomBLEScanner Class Implementation ---

void CustomBLEScanner::setup() {
  ESP_LOGD(TAG, "Setting up Custom BLE Scanner, RSSI threshold: %d dBm", rssi_threshold_);
  ESP_LOGD(TAG, "BTHome v2 Service UUID: %s", BTHOME_V2_SERVICE_UUID.to_string().c_str());

  if (this->tracker_ != nullptr) {
    this->tracker_->register_listener(this);
    ESP_LOGCONFIG(TAG, "Registered as listener to ESP32 BLE Tracker");
  } else {
    ESP_LOGE(TAG, "ESP32 BLE Tracker not set! Make sure 'esp32_ble_id' is configured correctly in YAML.");
  }
}

void CustomBLEScanner::loop() {
  if (mqtt::global_mqtt_client == nullptr || !mqtt::global_mqtt_client->is_connected()) {
    // Don't publish if MQTT is not connected
    return;
  }

  // Generic device data publishing (if still desired)
  if (millis() - last_generic_publish_time_ >= generic_publish_interval_ms_) {
    last_generic_publish_time_ = millis();
    ESP_LOGD(TAG, "Publishing generic BLE device data to MQTT...");

    for (auto const& [mac_address_u64, device_info] : known_ble_devices_) {
      std::string mac_address_str = mac_address_to_string(mac_address_u64);
      std::string mac_address_clean = mac_address_to_clean_string(mac_address_u64);

      std::string topic = "esphome/" + App.get_name() + "/ble/generic/" + mac_address_clean + "/state";

      DynamicJsonDocument doc(512);
      doc["mac"] = mac_address_str;
      doc["rssi"] = device_info.last_rssi;
      doc["last_seen"] = (millis() - device_info.last_advertisement_time) / 1000.0f;

      std::string payload;
      serializeJson(doc, payload);

      mqtt::global_mqtt_client->publish(topic, payload.c_str(), payload.length(), 0, true); // Retain=true
      ESP_LOGV(TAG, "Published generic data for %s to %s: %s", mac_address_str.c_str(), topic.c_str(), payload.c_str());
    }
  }

  // NEW: Periodically send discovery messages for all BTHome devices
  if (millis() - last_bthome_discovery_time_ >= BTHOME_DISCOVERY_INTERVAL_MS) {
    last_bthome_discovery_time_ = millis();
    this->send_all_bthome_discovery_messages();
  }
}

bool CustomBLEScanner::parse_device(const esp32_ble_tracker::ESPBTDevice &device) {
  const esp_ble_gap_cb_param_t::ble_scan_result_evt_param &scan_result = device.get_scan_result();
  std::vector<uint8_t> raw_ad_vector;
  if (scan_result.ble_adv != nullptr && scan_result.adv_data_len > 0) {
    raw_ad_vector.assign(scan_result.ble_adv, scan_result.ble_adv + scan_result.adv_data_len);
  }
  std::string raw_data_str = format_vector_to_hex(raw_ad_vector);

  // This block is for logging, only triggers once per second for efficiency
  if (millis() - last_log_time_ >= 1000) {
    ESP_LOGD(TAG, "BLE Advertisement received - MAC: %s, RSSI: %d dBm, RAW: %s",
             device.address_str().c_str(), device.get_rssi(), raw_data_str.c_str());
    last_log_time_ = millis();

    if (this->ble_raw_data_sensor_ != nullptr) {
      this->ble_raw_data_sensor_->publish_state(device.address_str() + " | RSSI: " + std::to_string(device.get_rssi()) + " | AD: " + raw_data_str);
    }
  }

  if (device.get_rssi() < rssi_threshold_) {
    ESP_LOGV(TAG, "Device %s skipped due to low RSSI: %d dBm", device.address_str().c_str(), device.get_rssi());
    return false;
  }

  bool is_bthome_v2_device = false;
  for (const auto &service_data_entry : device.get_service_datas()) {
    if (service_data_entry.uuid == BTHOME_V2_SERVICE_UUID) {
      is_bthome_v2_device = true;
      break;
    }
  }

  if (is_bthome_v2_device) {
    ESP_LOGD(TAG, "Found BTHome v2 device %s! Processing data...", device.address_str().c_str());
    return parse_bthome_v2_device(device); // This function will now manage BTHomeDevice objects
  }

  ESP_LOGV(TAG, "Storing generic device %s, RSSI: %d dBm", device.address_str().c_str(), device.get_rssi());
  BLEDeviceInfo info;
  info.last_advertisement_time = millis();
  info.last_rssi = device.get_rssi();
  known_ble_devices_[device.address_uint64()] = info;

  return true;
}

// NEW: Function to send discovery messages for a single BTHome device
void CustomBLEScanner::send_bthome_discovery_messages_for_device(BTHomeDevice *device_obj) {
    if (mqtt::global_mqtt_client == nullptr || !mqtt::global_mqtt_client->is_connected()) {
        ESP_LOGW(TAG, "MQTT not connected, skipping discovery for %s", mac_address_to_string(device_obj->address).c_str());
        return;
    }

    std::string node_name = App.get_name();
    // Use the overloaded mac_address_to_clean_string
    std::string mac_address_clean = mac_address_to_clean_string(device_obj->address);

    // HA Device Identifier: e.g., bthome-decoder8_38_1f_8d_58_11_57
    std::string ha_device_identifier = node_name + "_" + mac_address_clean;

    // Get the name for HA: using the BTHomeDevice's stored name
    std::string ha_device_name = device_obj->get_ha_device_name();

    // Final fallback if, for any reason, the name is still empty (shouldn't happen with proper init)
    if (ha_device_name.empty()) {
        ha_device_name = "BTHome " + mac_address_to_string(device_obj->address);
        ESP_LOGW(TAG, "HA device name for %s was empty. Falling back to MAC-based name: '%s'",
                 mac_address_to_string(device_obj->address).c_str(), ha_device_name.c_str());
    }

    ESP_LOGD(TAG, "DEBUG: HA Device Name for discovery: '%s'", ha_device_name.c_str());

    // Common 'device' block for Home Assistant
    DynamicJsonDocument device_doc(512);
    JsonArray identifiers = device_doc.createNestedArray("identifiers");
    identifiers.add(ha_device_identifier);
    device_doc["name"] = ha_device_name;
    device_doc["mdl"] = "BTHome v2 Device";
    device_doc["mf"] = "ESPHome BLE Gateway";
    // device_doc["sw"] = ESPHOME_VERSION; // Still removed

    // Base state topic for JSON payload: esphome/[node_name]/bthome/[mac_address_clean]/state
    std::string base_state_topic = "esphome/" + node_name + "/bthome/" + mac_address_clean + "/state";
    const std::string discovery_prefix = mqtt::global_mqtt_client->get_discovery_info().prefix;

    // Lambda to send sensor discovery messages
    auto send_sensor_discovery = [&](const std::string& sensor_type, const std::string& unit, const std::string& dev_class, const std::string& state_class) {
        DynamicJsonDocument sensor_config_doc(512);
        sensor_config_doc["name"] = sensor_type;
        std::string entity_id_clean = to_lower_string(sensor_type);
        
        sensor_config_doc["unique_id"] = ha_device_identifier + "_" + entity_id_clean; // Unique ID for entity

        sensor_config_doc["state_topic"] = base_state_topic;
        sensor_config_doc["value_template"] = "{{ value_json." + entity_id_clean + " }}"; 

        sensor_config_doc["unit_of_measurement"] = unit;
        sensor_config_doc["device_class"] = dev_class;
        sensor_config_doc["state_class"] = state_class;
        sensor_config_doc["expire_after"] = 300; // 5 minutes, keep devices active
        
        sensor_config_doc["device"] = device_doc; // Embed the common device block

        std::string discovery_topic = discovery_prefix + "/sensor/" + ha_device_identifier + "/" + entity_id_clean + "/config";
        std::string payload;
        serializeJson(sensor_config_doc, payload);

        mqtt::global_mqtt_client->publish(discovery_topic, payload.c_str(), payload.length(), 0, true); // Retain=true
        ESP_LOGD(TAG, "Sent %s discovery for %s (HA Device ID: %s)", sensor_type.c_str(), mac_address_to_string(device_obj->address).c_str(), ha_device_identifier.c_str());
    };

    // Send discovery for all supported BTHome measurement types
    send_sensor_discovery("Temperature", "°C", "temperature", "measurement");
    send_sensor_discovery("Humidity", "%", "humidity", "measurement");
    send_sensor_discovery("Battery", "%", "battery", "measurement");
    send_sensor_discovery("Voltage", "V", "voltage", "measurement");
}

// NEW: Main function to refresh discovery for all known BTHome devices
void CustomBLEScanner::send_all_bthome_discovery_messages() {
  if (mqtt::global_mqtt_client == nullptr || !mqtt::global_mqtt_client->is_connected()) {
    ESP_LOGW(TAG, "MQTT not connected, skipping global BTHome discovery scan.");
    return;
  }

  ESP_LOGD(TAG, "Starting global BTHome discovery refresh for %zu devices.", bthome_devices_.size());
  for (auto const& [mac_address_u64, device_obj] : bthome_devices_) {
    // Optional: Only send discovery for devices seen recently, e.g., within the last 10 minutes (600000ms)
    // if (millis() - device_obj->last_seen_millis_ < 600000) {
      this->send_bthome_discovery_messages_for_device(device_obj);
    // } else {
    //   ESP_LOGV(TAG, "Skipping discovery for inactive BTHome device %s", mac_address_to_string(device_obj->address).c_str());
    // }
  }
}

bool CustomBLEScanner::parse_bthome_v2_device(const esp32_ble_tracker::ESPBTDevice &device) {
  if (mqtt::global_mqtt_client == nullptr || !mqtt::global_mqtt_client->is_connected()) {
    ESP_LOGW(TAG, "MQTT not connected, skipping BTHome parsing for %s", device.address_str().c_str());
    return false;
  }

  const std::vector<esp32_ble_tracker::ServiceData>& service_data_entries = device.get_service_datas();
  const std::vector<uint8_t>* bthome_data_ptr = nullptr;

  for (const auto &entry : service_data_entries) {
    if (entry.uuid == BTHOME_V2_SERVICE_UUID) {
      bthome_data_ptr = &entry.data;
      break;
    }
  }

  if (bthome_data_ptr == nullptr) {
    ESP_LOGW(TAG, "BTHome v2 device %s missing FCD2 service data key after initial check! This should not happen.", device.address_str().c_str());
    return false;
  }
  const std::vector<uint8_t>& bthome_data = *bthome_data_ptr;


  if (bthome_data.size() < 2) {
    ESP_LOGW(TAG, "BTHome v2 device %s service data too short (%zu bytes). Payload: %s",
             device.address_str().c_str(), bthome_data.size(), format_vector_to_hex(bthome_data).c_str());
    return false;
  }

  uint8_t flags = bthome_data[0];
  bool encrypted = (flags & 0x01) != 0;
  if (encrypted) {
    ESP_LOGW(TAG, "BTHome v2 device %s is encrypted! Decryption not implemented in this component.", device.address_str().c_str());
    return false;
  }

  uint64_t device_mac_u64 = device.address_uint64();
  std::string device_mac_str = device.address_str(); // e.g. 38:1F:8D:58:11:57

  // --- Manage BTHomeDevice instances ---
  auto it = this->bthome_devices_.find(device_mac_u64);
  BTHomeDevice *bthome_dev;

  if (it == this->bthome_devices_.end()) {
    // New BTHome device detected
    ESP_LOGI(TAG, "First time seeing BTHome v2 device: %s. Creating new BTHomeDevice object.", device_mac_str.c_str());
    // FIX: Call the address() method to get the pointer
    bthome_dev = new BTHomeDevice(device.address()); // Create new BTHomeDevice
    this->bthome_devices_[device_mac_u64] = bthome_dev; // Store it in the map

    // Set initial HA name (MAC-based) for new devices
    bthome_dev->update_ha_device_name("BTHome " + mac_address_to_string(bthome_dev->address)); // Use overloaded function
    
    // Send discovery immediately for new devices
    this->send_bthome_discovery_messages_for_device(bthome_dev);
  } else {
    // Existing BTHome device
    bthome_dev = it->second;
  }

  // --- Update HA device name if a better advertised name is found ---
  std::string received_advertised_name = device.get_name(); // Get the advertised name from this scan
  bthome_dev->update_ha_device_name(received_advertised_name); // Call the update method

  // Update last seen time for this device
  bthome_dev->last_seen_millis_ = millis();

  // --- Parse Measurements and Publish State ---
  DynamicJsonDocument state_doc(512); // Increased to 512
  size_t offset = 1;

  while (offset < bthome_data.size()) {
    uint8_t type = bthome_data[offset];
    offset++;

    float value = NAN; // Initialize value to Not-a-Number
    bool parsed_successfully = true; // Flag to track if measurement was successfully parsed

    switch (type) {
      case BTHOME_PACKET_ID: {
        if (offset + 1 > bthome_data.size()) { parsed_successfully = false; break;}
        uint8_t raw_packet_id = bthome_data[offset];
        ESP_LOGD(TAG, "  BTHome Packet ID: 0x%02X", raw_packet_id);
        offset += 1;
        break;
      }
      case BTHOME_MEASUREMENT_TEMPERATURE: {
        if (offset + 2 > bthome_data.size()) { parsed_successfully = false; break;}
        int16_t raw_temp = (int16_t) (bthome_data[offset] | (bthome_data[offset+1] << 8));
        value = raw_temp * 0.01f;
        ESP_LOGD(TAG, "  BTHome Temperature: %.2f °C", value);
        state_doc["temperature"] = round(value * 100) / 100.0;
        if (this->bthome_temperature_sensor_ != nullptr) {
          this->bthome_temperature_sensor_->publish_state(value);
        }
        offset += 2;
        break;
      }
      case BTHOME_MEASUREMENT_HUMIDITY: {
        if (offset + 2 > bthome_data.size()) { parsed_successfully = false; break;}
        uint16_t raw_hum = (uint16_t) (bthome_data[offset] | (bthome_data[offset+1] << 8));
        value = raw_hum * 0.01f;
        ESP_LOGD(TAG, "  BTHome Humidity: %.2f %%", value);
        state_doc["humidity"] = round(value * 100) / 100.0;
        if (this->bthome_humidity_sensor_ != nullptr) {
          this->bthome_humidity_sensor_->publish_state(value);
        }
        offset += 2;
        break;
      }
      case BTHOME_MEASUREMENT_BATTERY: {
        if (offset + 1 > bthome_data.size()) { parsed_successfully = false; break;}
        uint8_t bat_percent = bthome_data[offset];
        value = (float)bat_percent;
        ESP_LOGD(TAG, "  BTHome Battery: %.0f %%", value);
        state_doc["battery"] = (int)value;
        if (this->bthome_battery_sensor_ != nullptr) {
          this->bthome_battery_sensor_->publish_state(value);
        }
        offset += 1;
        break;
      }
      case BTHOME_MEASUREMENT_VOLTAGE: {
        if (offset + 2 > bthome_data.size()) { parsed_successfully = false; break;}
        uint16_t raw_voltage_mv = (uint16_t) (bthome_data[offset] | (bthome_data[offset+1] << 8));
        value = raw_voltage_mv * 0.001f;
        ESP_LOGD(TAG, "  BTHome Voltage: %.3f V", value);
        state_doc["voltage"] = round(value * 1000) / 1000.0;
        if (this->bthome_voltage_sensor_ != nullptr) {
            this->bthome_voltage_sensor_->publish_state(value);
        }
        offset += 2;
        break;
      }
      default: {
        ESP_LOGW(TAG, "Unknown BTHome v2 data type: 0x%02X for %s at offset %zu. Full payload: %s",
                 type, device.address_str().c_str(), offset - 1, format_vector_to_hex(bthome_data).c_str());
        offset = bthome_data.size(); // Skip remaining if unknown type encountered
        parsed_successfully = false;
        break;
      }
    }
     // If a measurement was successfully parsed, store it in the BTHomeDevice's measurements map
    if (parsed_successfully && !std::isnan(value)) {
        bthome_dev->measurements[type] = {
            .name = to_lower_string(BTHomeDataTypeToString(type)), // Convert type enum to string, make it lowercase
            .value = value,
            .unit = BTHomeDataTypeToUnit(type),
            .device_class = BTHomeDataTypeToDeviceClass(type),
            .state_class = BTHomeDataTypeToStateClass(type)
        };
    }
  }

  // Publish state to MQTT
  std::string state_topic = "esphome/" + App.get_name() + "/bthome/" + mac_address_to_clean_string(device_mac_u64) + "/state";
  std::string state_payload;
  serializeJson(state_doc, state_payload);

  if (!state_doc.isNull() && state_doc.size() > 0) {
    mqtt::global_mqtt_client->publish(state_topic, state_payload.c_str(), state_payload.length(), 0, false);
    ESP_LOGV(TAG, "Published BTHome state for %s to %s: %s", device_mac_str.c_str(), state_topic.c_str(), state_payload.c_str());
  } else {
    ESP_LOGD(TAG, "No BTHome measurements parsed for %s, skipping MQTT state publish.", device_mac_str.c_str());
  }

  return true;
}

void CustomBLEScanner::dump_config() {
  ESP_LOGCONFIG(TAG, "Custom BLE Scanner:");
  ESP_LOGCONFIG(TAG, "  RSSI Threshold: %d dBm", rssi_threshold_);
  ESP_LOGCONFIG(TAG, "  Generic MQTT Publish Interval: %u ms", generic_publish_interval_ms_);
  ESP_LOGCONFIG(TAG, "  BTHome Discovery Interval: %u ms", BTHOME_DISCOVERY_INTERVAL_MS);
  if (this->tracker_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  Attached to ESP32 BLE Tracker");
  }
  if (this->ble_raw_data_sensor_ != nullptr) {
      ESP_LOGCONFIG(TAG, "  BLE Raw Data Text Sensor: %s", this->ble_raw_data_sensor_->get_name().c_str());
  }
  if (this->bthome_temperature_sensor_ != nullptr) {
      ESP_LOGCONFIG(TAG, "  BTHome Temperature Sensor: %s", this->bthome_temperature_sensor_->get_name().c_str());
  }
  if (this->bthome_humidity_sensor_ != nullptr) {
      ESP_LOGCONFIG(TAG, "  BTHome Humidity Sensor: %s", this->bthome_humidity_sensor_->get_name().c_str());
  }
  if (this->bthome_battery_sensor_ != nullptr) {
      ESP_LOGCONFIG(TAG, "  BTHome Battery Sensor: %s", this->bthome_battery_sensor_->get_name().c_str());
  }
  if (this->bthome_voltage_sensor_ != nullptr) {
      ESP_LOGCONFIG(TAG, "  BTHome Voltage Sensor: %s", this->bthome_voltage_sensor_->get_name().c_str());
  }
  ESP_LOGCONFIG(TAG, "  BTHome devices will be dynamically discovered via MQTT.");
}

void CustomBLEScanner::set_rssi_threshold(int rssi) {
  rssi_threshold_ = rssi;
  ESP_LOGD(TAG, "RSSI threshold set to %d dBm", rssi_threshold_);
}


// --- Helper functions for BTHomeMeasurement (definitions) ---
std::string BTHomeDataTypeToString(uint8_t type) {
    switch (type) {
        case BTHOME_MEASUREMENT_TEMPERATURE: return "Temperature";
        case BTHOME_MEASUREMENT_HUMIDITY: return "Humidity";
        case BTHOME_MEASUREMENT_BATTERY: return "Battery";
        case BTHOME_MEASUREMENT_VOLTAGE: return "Voltage";
        default: return "Unknown";
    }
}

std::string BTHomeDataTypeToUnit(uint8_t type) {
    switch (type) {
        case BTHOME_MEASUREMENT_TEMPERATURE: return "°C";
        case BTHOME_MEASUREMENT_HUMIDITY: return "%";
        case BTHOME_MEASUREMENT_BATTERY: return "%";
        case BTHOME_MEASUREMENT_VOLTAGE: return "V";
        default: return "";
    }
}

std::string BTHomeDataTypeToDeviceClass(uint8_t type) {
    switch (type) {
        case BTHOME_MEASUREMENT_TEMPERATURE: return "temperature";
        case BTHOME_MEASUREMENT_HUMIDITY: return "humidity";
        case BTHOME_MEASUREMENT_BATTERY: return "battery";
        case BTHOME_MEASUREMENT_VOLTAGE: return "voltage";
        default: return "";
    }
}

std::string BTHomeDataTypeToStateClass(uint8_t type) {
    switch (type) {
        case BTHOME_MEASUREMENT_TEMPERATURE: return "measurement";
        case BTHOME_MEASUREMENT_HUMIDITY: return "measurement";
        case BTHOME_MEASUREMENT_BATTERY: return "measurement";
        case BTHOME_MEASUREMENT_VOLTAGE: return "measurement";
        default: return "";
    }
}

} // namespace custom_ble_scanner
} // namespace esphome