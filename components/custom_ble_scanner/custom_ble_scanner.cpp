// custom_components/custom_ble_scanner/custom_ble_scanner.cpp
#include "custom_ble_scanner.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h" 
#include "esphome/core/application.h" 
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/mqtt/mqtt_client.h"
#include "esphome/components/json/json_util.h"
#include <cmath>
#include <algorithm>
#include <cctype>
#include <string>
#include "esphome/components/esp32_ble_tracker/esp32_ble_tracker.h"

namespace esphome {
namespace custom_ble_scanner {

static const char *TAG = "custom_ble_scanner";

// Helper function to convert string to lowercase
std::string to_lower_string(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(),
                  [](unsigned char c){ return std::tolower(c); });
  return s;
}

// Helper function to convert MAC address to string
std::string mac_address_to_string(uint64_t mac) {
  char buffer[18];
  snprintf(buffer, sizeof(buffer), "%02X:%02X:%02X:%02X:%02X:%02X",
           (uint8_t)(mac >> 40), (uint8_t)(mac >> 32), (uint8_t)(mac >> 24),
           (uint8_t)(mac >> 16), (uint8_t)(mac >> 8), (uint8_t)(mac >> 0));
  return std::string(buffer);
}

// Overload for uint8_t array
std::string mac_address_to_string(const uint8_t* mac_array) {
  char buffer[18];
  snprintf(buffer, sizeof(buffer), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac_array[0], mac_array[1], mac_array[2], mac_array[3], mac_array[4], mac_array[5]);
  return std::string(buffer);
}

// Helper function to convert MAC address to clean string for MQTT
std::string mac_address_to_clean_string(uint64_t mac) {
  char buffer[18];
  snprintf(buffer, sizeof(buffer), "%02x_%02x_%02x_%02x_%02x_%02x",
           (uint8_t)(mac >> 40), (uint8_t)(mac >> 32), (uint8_t)(mac >> 24),
           (uint8_t)(mac >> 16), (uint8_t)(mac >> 8), (uint8_t)(mac >> 0));
  return std::string(buffer);
}

// Overload for uint8_t array
std::string mac_address_to_clean_string(const uint8_t* mac_array) {
  char buffer[18];
  snprintf(buffer, sizeof(buffer), "%02x_%02x_%02x_%02x_%02x_%02x",
           mac_array[0], mac_array[1], mac_array[2], mac_array[3], mac_array[4], mac_array[5]);
  return std::string(buffer);
}

std::string format_vector_to_hex(const std::vector<uint8_t>& data) {
  if (data.empty()) return "";
  return esphome::format_hex_pretty(data);
}

std::string format_time_ago(uint32_t seconds) {
  if (seconds < 60) return std::to_string(seconds) + "s ago";
  if (seconds < 3600) return std::to_string(seconds / 60) + "m " + std::to_string(seconds % 60) + "s ago";
  if (seconds < 86400) return std::to_string(seconds / 3600) + "h " + std::to_string((seconds % 3600) / 60) + "m ago";
  return std::to_string(seconds / 86400) + "d " + std::to_string((seconds % 86400) / 3600) + "h ago";
}

std::map<std::string, std::string> decode_manufacturer_data(uint16_t company_id, const std::vector<uint8_t>& data) {
  std::map<std::string, std::string> decoded;
  switch (company_id) {
    case 0x004C: {
      decoded["manufacturer"] = "Apple";
      if (data.size() < 2) { decoded["type"] = "Too Short"; return decoded; }
      uint8_t type = data[0];
      switch (type) {
        case 0x02: decoded["type"] = "iPhone/iPad/Mac"; break;
        case 0x03: decoded["type"] = "Watch"; break;
        case 0x05: decoded["type"] = "AirPods"; break;
        case 0x07: decoded["type"] = "HomePod"; break;
        case 0x09: decoded["type"] = "AirTag"; break;
        case 0x0C: decoded["type"] = "Find My"; break;
        case 0x10: decoded["type"] = "AirDrop"; break;
        default: decoded["type"] = "Unknown"; break;
      }
      break;
    }
    case 0x0075: {
      decoded["manufacturer"] = "Samsung";
      if (data.size() < 1) { decoded["type"] = "Too Short"; return decoded; }
      if (data[0] == 0x54) { decoded["type"] = "SmartThings"; } 
      else { decoded["type"] = "Unknown"; }
      break;
    }
  }
  return decoded;
}

// --- CustomBLEScanner Class Implementation ---

// NEW: Helper function and setter for the ignore list
uint64_t mac_address_to_uint64(const std::string &mac_str) {
    uint64_t mac = 0;
    const char *p = mac_str.c_str();
    for (int i = 0; i < 6; i++) {
        mac = (mac << 8) | (uint64_t)strtol(p, (char**)&p, 16);
        if (i < 5) p++; // Skip the ':' or '-'
    }
    return mac;
}

void CustomBLEScanner::set_ignore_mac_addresses(const std::vector<std::string> &macs) {
  for (const auto &mac_str : macs) {
    this->ignore_mac_addresses_.insert(mac_address_to_uint64(mac_str));
  }
}

void CustomBLEScanner::setup() {
  ESP_LOGD(TAG, "Setting up Custom BLE Scanner, RSSI threshold: %d dBm", this->rssi_threshold_);
  if (this->tracker_ != nullptr) {
    this->tracker_->register_listener(this);
  } else {
    ESP_LOGE(TAG, "ESP32 BLE Tracker not set!");
  }
}

void CustomBLEScanner::loop() {
  if (millis() - this->last_prune_time_ > 300000) {
    this->last_prune_time_ = millis();
    this->prune_stale_devices();
  }
  
  if (mqtt::global_mqtt_client == nullptr || !mqtt::global_mqtt_client->is_connected()) return;

  if (this->generic_scanner_enabled_ && (millis() - this->last_generic_publish_time_ >= this->generic_publish_interval_ms_)) {
    this->last_generic_publish_time_ = millis();
    ESP_LOGD(TAG, "Publishing generic BLE device data to MQTT...");
    for (auto const& [mac_address_u64, device_info] : this->known_ble_devices_) {
      std::string mac_address_clean = mac_address_to_clean_string(mac_address_u64);
      std::string topic = "esphome/" + App.get_name() + "/ble/generic/" + mac_address_clean + "/state";

      JsonDocument doc;
      doc["mac"] = mac_address_to_string(mac_address_u64);
      doc["name"] = device_info.name;
      doc["rssi"] = device_info.last_rssi;
      doc["last_seen_ago"] = format_time_ago((millis() - device_info.last_advertisement_time) / 1000);
      doc["manufacturer_data"] = device_info.manufacturer_data;
      
      if (!device_info.decoded_info.empty()) {
          JsonObject decoded_obj = doc["decoded_info"].to<JsonObject>();
          for (const auto& [key, value] : device_info.decoded_info) {
              decoded_obj[key] = value;
          }
      }

      std::string payload;
      serializeJson(doc, payload);
      mqtt::global_mqtt_client->publish(topic, payload.c_str(), payload.length(), 0, true);
    }
  }

  if (millis() - this->last_bthome_discovery_time_ >= this->bthome_discovery_interval_ms_) {
    this->last_bthome_discovery_time_ = millis();
    this->send_all_bthome_discovery_messages();
  }
}

bool CustomBLEScanner::parse_device(const esp32_ble_tracker::ESPBTDevice &device) {
  // NEW: Check the global ignore list first.
  if (this->ignore_mac_addresses_.count(device.address_uint64())) {
    ESP_LOGV(TAG, "Device %s is on the ignore list, skipping.", device.address_str().c_str());
    return false;
  }
  
  // Check for BTHome devices first
  for (const auto &service_data_entry : device.get_service_datas()) {
    if (service_data_entry.uuid == BTHOME_V2_SERVICE_UUID) {
      ESP_LOGD(TAG, "Found BTHome v2 device %s! Processing data...", device.address_str().c_str());
      return parse_bthome_v2_device(device);
    }
  }

  if (!this->generic_scanner_enabled_) return false;
  
  if (device.get_rssi() < this->rssi_threshold_) {
    ESP_LOGV(TAG, "Generic device %s skipped due to low RSSI.", device.address_str().c_str());
    return false;
  }
  
  BLEDeviceInfo info;
  info.last_advertisement_time = millis();
  info.last_rssi = device.get_rssi();
  
  std::string device_name = device.get_name();
  info.name = device_name.empty() ? "N/A" : device_name;
  
  std::string manuf_data_str = "";
  std::map<std::string, std::string> all_decoded_info;
  for (const auto &manuf_data : device.get_manufacturer_datas()) {
    uint16_t company_id = manuf_data.uuid.get_uuid().uuid.uuid16;
    
    char buffer[7];
    snprintf(buffer, sizeof(buffer), "0x%04X", company_id);
    manuf_data_str += buffer;
    manuf_data_str += ": ";
    manuf_data_str += format_hex_pretty(manuf_data.data);
    manuf_data_str += " | ";
    
    auto decoded_part = decode_manufacturer_data(company_id, manuf_data.data);
    if (!decoded_part.empty()) {
        all_decoded_info.insert(decoded_part.begin(), decoded_part.end());
    }
  }
  info.manufacturer_data = manuf_data_str;
  info.decoded_info = all_decoded_info;
  
  this->known_ble_devices_[device.address_uint64()] = info;

  return true;
}

void CustomBLEScanner::send_bthome_discovery_messages_for_device(BTHomeDevice *device_obj) {
    if (mqtt::global_mqtt_client == nullptr || !mqtt::global_mqtt_client->is_connected()) return;

    std::string node_name = App.get_name();
    std::string mac_address_clean = mac_address_to_clean_string(device_obj->address);
    std::string ha_device_identifier = node_name + "_" + mac_address_clean;
    std::string ha_device_name = device_obj->get_ha_device_name();

    if (ha_device_name.empty()) {
        ha_device_name = "BTHome " + mac_address_to_string(device_obj->address);
    }

    JsonDocument device_doc;
    JsonArray identifiers = device_doc["identifiers"].to<JsonArray>();
    identifiers.add(ha_device_identifier);
    device_doc["name"] = ha_device_name;
    device_doc["mdl"] = "BTHome v2 Device";
    device_doc["mf"] = "ESPHome BLE Gateway";

    std::string base_state_topic = "esphome/" + node_name + "/bthome/" + mac_address_clean + "/state";
    const std::string discovery_prefix = mqtt::global_mqtt_client->get_discovery_info().prefix;

    auto send_sensor_discovery = [&](const BTHomeMeasurement& measurement) {
        JsonDocument sensor_config_doc;
        std::string entity_id_clean = to_lower_string(measurement.name);
        
        sensor_config_doc["name"] = measurement.name;
        sensor_config_doc["unique_id"] = ha_device_identifier + "_" + entity_id_clean;
        sensor_config_doc["state_topic"] = base_state_topic;
        sensor_config_doc["value_template"] = "{{ value_json." + entity_id_clean + " }}"; 
        sensor_config_doc["unit_of_measurement"] = measurement.unit;
        sensor_config_doc["device_class"] = measurement.device_class;
        sensor_config_doc["state_class"] = measurement.state_class;
        sensor_config_doc["expire_after"] = this->bthome_discovery_interval_ms_ / 1000;
        sensor_config_doc["device"] = device_doc;

        std::string discovery_topic = discovery_prefix + "/sensor/" + ha_device_identifier + "/" + entity_id_clean + "/config";
        std::string payload;
        serializeJson(sensor_config_doc, payload);

        mqtt::global_mqtt_client->publish(discovery_topic, payload.c_str(), payload.length(), 0, true);
    };

    for (const auto& [type_id, measurement] : device_obj->measurements) {
        send_sensor_discovery(measurement);
    }
}

void CustomBLEScanner::send_all_bthome_discovery_messages() {
  if (mqtt::global_mqtt_client == nullptr || !mqtt::global_mqtt_client->is_connected()) return;
  ESP_LOGD(TAG, "Sending periodic BTHome discovery messages for %zu devices.", this->bthome_devices_.size());
  for (auto const& [mac_address_u64, device_obj] : this->bthome_devices_) {
    if (millis() - device_obj->last_seen_millis_ < 600000) {
      this->send_bthome_discovery_messages_for_device(device_obj);
    }
  }
}

bool CustomBLEScanner::parse_bthome_v2_device(const esp32_ble_tracker::ESPBTDevice &device) {
  const std::vector<uint8_t>* bthome_data_ptr = nullptr;
  for (const auto &entry : device.get_service_datas()) {
    if (entry.uuid == BTHOME_V2_SERVICE_UUID) {
      bthome_data_ptr = &entry.data;
      break;
    }
  }

  if (bthome_data_ptr == nullptr) return false;
  const std::vector<uint8_t>& bthome_data = *bthome_data_ptr;

  if (bthome_data.size() < 2 || (bthome_data[0] & 0x01) != 0) return false;

  uint64_t device_mac_u64 = device.address_uint64();
  BTHomeDevice *bthome_dev;
  auto it = this->bthome_devices_.find(device_mac_u64);
  bool is_new_device = (it == this->bthome_devices_.end());

  if (is_new_device) {
    ESP_LOGI(TAG, "First time seeing BTHome v2 device: %s.", device.address_str().c_str());
    bthome_dev = new BTHomeDevice(device.address());
    this->bthome_devices_[device_mac_u64] = bthome_dev;
    bthome_dev->update_ha_device_name("BTHome " + mac_address_to_string(bthome_dev->address));
  } else {
    bthome_dev = it->second;
  }

  std::string bthome_device_name = device.get_name();
  if (!bthome_device_name.empty()) {
      bthome_dev->update_ha_device_name(bthome_device_name);
  }
  bthome_dev->last_seen_millis_ = millis();

  JsonDocument state_doc;
  size_t offset = 1;
  bool new_measurement_found = false;

  while (offset < bthome_data.size()) {
    uint8_t type = bthome_data[offset];
    offset++;
    float value = NAN;
    bool parsed_successfully = true;
    
    #define PARSE_BTHOME_FIELD(TYPE, C_TYPE, SIZE, FACTOR, JSON_KEY) \
      case TYPE: { \
        if (offset + SIZE > bthome_data.size()) break; \
        uint32_t raw_value = bthome_data[offset]; \
        if (SIZE > 1) raw_value |= (bthome_data[offset+1] << 8); \
        if (SIZE > 2) raw_value |= (bthome_data[offset+2] << 16); \
        if (SIZE > 3) raw_value |= (bthome_data[offset+3] << 24); \
        value = (float)(C_TYPE)raw_value * FACTOR; \
        state_doc[#JSON_KEY] = value; \
        offset += SIZE; \
        break; \
      }

    switch (type) {
      PARSE_BTHOME_FIELD(BTHOME_MEASUREMENT_BATTERY, uint8_t, 1, 1.0f, battery)
      PARSE_BTHOME_FIELD(BTHOME_MEASUREMENT_TEMPERATURE, int16_t, 2, 0.01f, temperature)
      PARSE_BTHOME_FIELD(BTHOME_MEASUREMENT_HUMIDITY, uint16_t, 2, 0.01f, humidity)
      PARSE_BTHOME_FIELD(BTHOME_MEASUREMENT_PRESSURE, uint32_t, 3, 0.01f, pressure)
      PARSE_BTHOME_FIELD(BTHOME_MEASUREMENT_ILLUMINANCE, uint32_t, 3, 0.01f, illuminance)
      PARSE_BTHOME_FIELD(BTHOME_MEASUREMENT_MASS_KG, uint16_t, 2, 0.01f, mass_kg)
      PARSE_BTHOME_FIELD(BTHOME_MEASUREMENT_MASS_LB, uint16_t, 2, 0.01f, mass_lb)
      PARSE_BTHOME_FIELD(BTHOME_MEASUREMENT_DEWPOINT, int16_t, 2, 0.01f, dewpoint)
      PARSE_BTHOME_FIELD(BTHOME_MEASUREMENT_COUNT_S, uint8_t, 1, 1.0f, count)
      PARSE_BTHOME_FIELD(BTHOME_MEASUREMENT_COUNT_LEGACY, uint16_t, 2, 1.0f, count)
      PARSE_BTHOME_FIELD(BTHOME_MEASUREMENT_COUNT_M, uint16_t, 2, 1.0f, count)
      PARSE_BTHOME_FIELD(BTHOME_MEASUREMENT_COUNT_L, uint32_t, 4, 1.0f, count)
      PARSE_BTHOME_FIELD(BTHOME_MEASUREMENT_ENERGY, uint32_t, 3, 0.001f, energy)
      PARSE_BTHOME_FIELD(BTHOME_MEASUREMENT_POWER, int32_t, 3, 0.01f, power)
      PARSE_BTHOME_FIELD(BTHOME_MEASUREMENT_VOLTAGE, uint16_t, 2, 0.001f, voltage)
      PARSE_BTHOME_FIELD(BTHOME_MEASUREMENT_DISTANCE, uint16_t, 2, 0.1f, distance)
      PARSE_BTHOME_FIELD(BTHOME_MEASUREMENT_CO2, uint16_t, 2, 1.0f, co2)
      PARSE_BTHOME_FIELD(BTHOME_MEASUREMENT_VOC, uint16_t, 2, 1.0f, voc)
      case BTHOME_PACKET_ID: { offset += 1; break; }
      default: {
        ESP_LOGW(TAG, "Unknown BTHome v2 data type: 0x%02X for device %s", type, device.address_str().c_str());
        offset = bthome_data.size();
        break;
      }
    }

    if (!std::isnan(value)) {
        if (bthome_dev->measurements.find(type) == bthome_dev->measurements.end()) {
            new_measurement_found = true;
        }
        bthome_dev->measurements[type] = {
            .name = BTHomeDataTypeToString(type), .value = value, .unit = BTHomeDataTypeToUnit(type),
            .device_class = BTHomeDataTypeToDeviceClass(type), .state_class = BTHomeDataTypeToStateClass(type)
        };
    }
  }

  if (!state_doc.isNull() && state_doc.size() > 0) {
    std::string state_topic = "esphome/" + App.get_name() + "/bthome/" + mac_address_to_clean_string(device_mac_u64) + "/state";
    std::string state_payload;
    serializeJson(state_doc, state_payload);
    mqtt::global_mqtt_client->publish(state_topic, state_payload.c_str(), state_payload.length(), 0, false);
  }
  
  if (is_new_device || new_measurement_found) {
    this->send_bthome_discovery_messages_for_device(bthome_dev);
  }

  return true;
}

void CustomBLEScanner::prune_stale_devices() {
  const uint32_t now = millis();
  ESP_LOGD(TAG, "Pruning stale devices from memory (older than %u ms)...", this->prune_timeout_);
  int pruned_count = 0;
  for (auto it = this->known_ble_devices_.begin(); it != this->known_ble_devices_.end();) {
    if (now - it->second.last_advertisement_time > this->prune_timeout_) {
      it = this->known_ble_devices_.erase(it);
      pruned_count++;
    } else { ++it; }
  }
  if (pruned_count > 0) ESP_LOGD(TAG, "Pruned %d stale generic devices.", pruned_count);

  pruned_count = 0;
  for (auto it = this->bthome_devices_.begin(); it != this->bthome_devices_.end();) {
    if (now - it->second->last_seen_millis_ > this->prune_timeout_) {
      delete it->second;
      it = this->bthome_devices_.erase(it);
      pruned_count++;
    } else { ++it; }
  }
  if (pruned_count > 0) ESP_LOGD(TAG, "Pruned %d stale BTHome devices.", pruned_count);
}

void CustomBLEScanner::dump_config() {
  ESP_LOGCONFIG(TAG, "Custom BLE Scanner:");
  ESP_LOGCONFIG(TAG, "  RSSI Threshold: %d dBm", this->rssi_threshold_);
  ESP_LOGCONFIG(TAG, "  Generic Scanner Enabled: %s", this->generic_scanner_enabled_ ? "true" : "false");
  if (this->generic_scanner_enabled_) {
    ESP_LOGCONFIG(TAG, "  Generic Publish Interval: %u ms", this->generic_publish_interval_ms_);
  }
  ESP_LOGCONFIG(TAG, "  BTHome Discovery Interval: %u ms", this->bthome_discovery_interval_ms_);
  ESP_LOGCONFIG(TAG, "  Pruning Timeout: %u ms", this->prune_timeout_);
}

// --- Helper functions for BTHomeMeasurement (definitions) ---
std::string BTHomeDataTypeToString(uint8_t type) {
    switch (type) {
        case BTHOME_MEASUREMENT_BATTERY: return "Battery";
        case BTHOME_MEASUREMENT_TEMPERATURE: return "Temperature";
        case BTHOME_MEASUREMENT_HUMIDITY: return "Humidity";
        case BTHOME_MEASUREMENT_PRESSURE: return "Pressure";
        case BTHOME_MEASUREMENT_ILLUMINANCE: return "Illuminance";
        case BTHOME_MEASUREMENT_MASS_KG: return "Mass";
        case BTHOME_MEASUREMENT_MASS_LB: return "Mass Lb";
        case BTHOME_MEASUREMENT_DEWPOINT: return "Dewpoint";
        case BTHOME_MEASUREMENT_COUNT_S: return "Count";
        case BTHOME_MEASUREMENT_COUNT_M: return "Count M";
        case BTHOME_MEASUREMENT_COUNT_L: return "Count L";
        case BTHOME_MEASUREMENT_COUNT_LEGACY: return "Count";
        case BTHOME_MEASUREMENT_ENERGY: return "Energy";
        case BTHOME_MEASUREMENT_POWER: return "Power";
        case BTHOME_MEASUREMENT_VOLTAGE: return "Voltage";
        case BTHOME_MEASUREMENT_DISTANCE: return "Distance";
        case BTHOME_MEASUREMENT_CO2: return "CO2";
        case BTHOME_MEASUREMENT_VOC: return "VOC";
        case BTHOME_PACKET_ID: return "Packet ID";
        default: return "Unknown";
    }
}

std::string BTHomeDataTypeToUnit(uint8_t type) {
    switch (type) {
        case BTHOME_MEASUREMENT_BATTERY: return "%";
        case BTHOME_MEASUREMENT_TEMPERATURE: return "°C";
        case BTHOME_MEASUREMENT_HUMIDITY: return "%";
        case BTHOME_MEASUREMENT_PRESSURE: return "hPa";
        case BTHOME_MEASUREMENT_ILLUMINANCE: return "lx";
        case BTHOME_MEASUREMENT_MASS_KG: return "kg";
        case BTHOME_MEASUREMENT_MASS_LB: return "lb";
        case BTHOME_MEASUREMENT_DEWPOINT: return "°C";
        case BTHOME_MEASUREMENT_ENERGY: return "kWh";
        case BTHOME_MEASUREMENT_POWER: return "W";
        case BTHOME_MEASUREMENT_VOLTAGE: return "V";
        case BTHOME_MEASUREMENT_DISTANCE: return "m";
        case BTHOME_MEASUREMENT_CO2: return "ppm";
        case BTHOME_MEASUREMENT_VOC: return "µg/m³";
        default: return "";
    }
}

std::string BTHomeDataTypeToDeviceClass(uint8_t type) {
    switch (type) {
        case BTHOME_MEASUREMENT_BATTERY: return "battery";
        case BTHOME_MEASUREMENT_TEMPERATURE: return "temperature";
        case BTHOME_MEASUREMENT_HUMIDITY: return "humidity";
        case BTHOME_MEASUREMENT_PRESSURE: return "pressure";
        case BTHOME_MEASUREMENT_ILLUMINANCE: return "illuminance";
        case BTHOME_MEASUREMENT_MASS_KG: return "weight";
        case BTHOME_MEASUREMENT_MASS_LB: return "weight";
        case BTHOME_MEASUREMENT_DEWPOINT: return "temperature";
        case BTHOME_MEASUREMENT_ENERGY: return "energy";
        case BTHOME_MEASUREMENT_POWER: return "power";
        case BTHOME_MEASUREMENT_VOLTAGE: return "voltage";
        case BTHOME_MEASUREMENT_DISTANCE: return "distance";
        case BTHOME_MEASUREMENT_CO2: return "carbon_dioxide";
        case BTHOME_MEASUREMENT_VOC: return "volatile_organic_compounds";
        default: return "";
    }
}

std::string BTHomeDataTypeToStateClass(uint8_t type) {
    switch (type) {
        case BTHOME_MEASUREMENT_POWER:
        case BTHOME_MEASUREMENT_TEMPERATURE:
        case BTHOME_MEASUREMENT_HUMIDITY:
        case BTHOME_MEASUREMENT_PRESSURE:
        case BTHOME_MEASUREMENT_ILLUMINANCE:
        case BTHOME_MEASUREMENT_MASS_KG:
        case BTHOME_MEASUREMENT_MASS_LB:
        case BTHOME_MEASUREMENT_DEWPOINT:
        case BTHOME_MEASUREMENT_VOLTAGE:
        case BTHOME_MEASUREMENT_CO2:
        case BTHOME_MEASUREMENT_VOC:
        case BTHOME_MEASUREMENT_BATTERY:
        case BTHOME_MEASUREMENT_DISTANCE:
            return "measurement";
        case BTHOME_MEASUREMENT_COUNT_S:
        case BTHOME_MEASUREMENT_COUNT_M:
        case BTHOME_MEASUREMENT_COUNT_L:
        case BTHOME_MEASUREMENT_COUNT_LEGACY:
        case BTHOME_MEASUREMENT_ENERGY:
            return "total_increasing";
        default:
            return "";
    }
}

} // namespace custom_ble_scanner
} // namespace esphome
}