// custom_components/custom_ble_scanner/custom_ble_scanner.cpp
#include "custom_ble_scanner.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h" 
#include "esphome/core/application.h" 
#include "esphome/components/sensor/sensor.h"
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

// Helper function to format seconds into a human-readable string
std::string format_time_ago(uint32_t seconds) {
  if (seconds < 60) {
    return std::to_string(seconds) + "s ago";
  }
  if (seconds < 3600) {
    return std::to_string(seconds / 60) + "m " + std::to_string(seconds % 60) + "s ago";
  }
  if (seconds < 86400) {
    return std::to_string(seconds / 3600) + "h " + std::to_string((seconds % 3600) / 60) + "m ago";
  }
  return std::to_string(seconds / 86400) + "d " + std::to_string((seconds % 86400) / 3600) + "h ago";
}

// Helper function to decode manufacturer data
std::string decode_manufacturer_data(uint16_t company_id, const std::vector<uint8_t>& data) {
  switch (company_id) {
    case 0x004C: { // Apple
      if (data.size() < 2) return "Apple (Too Short)";
      uint8_t type = data[0];
      switch (type) {
        case 0x02: return "Apple Device (iPhone/iPad/Mac)";
        case 0x03: return "Apple Device (Watch)";
        case 0x05: return "Apple Device (AirPods)";
        case 0x07: return "Apple Device (HomePod)";
        case 0x09: return "Apple Device (AirTag)";
        case 0x0C: return "Apple Find My";
        case 0x10: return "Apple AirDrop";
        default: return "Apple (Unknown Type)";
      }
      break;
    }
    case 0x0075: { // Samsung
      if (data.size() < 1) return "Samsung (Too Short)";
      uint8_t type = data[0];
      if (type == 0x54) {
          return "Samsung SmartThings";
      }
      return "Samsung (Unknown Type)";
    }
    default:
      return "";
  }
  return "";
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
  // Periodically prune stale devices to prevent memory leaks
  if (millis() - last_prune_time_ > 300000) { // Prune every 5 minutes
    last_prune_time_ = millis();
    this->prune_stale_devices();
  }
  
  if (mqtt::global_mqtt_client == nullptr || !mqtt::global_mqtt_client->is_connected()) {
    return;
  }

  // Generic device data publishing
  if (millis() - last_generic_publish_time_ >= generic_publish_interval_ms_) {
    last_generic_publish_time_ = millis();
    ESP_LOGD(TAG, "Publishing generic BLE device data to MQTT...");

    for (auto const& [mac_address_u64, device_info] : known_ble_devices_) {
      std::string mac_address_str = mac_address_to_string(mac_address_u64);
      std::string mac_address_clean = mac_address_to_clean_string(mac_address_u64);
      std::string topic = "esphome/" + App.get_name() + "/ble/generic/" + mac_address_clean + "/state";

      JsonDocument doc;
      doc["mac"] = mac_address_str;
      doc["name"] = device_info.name;
      doc["rssi"] = device_info.last_rssi;
      doc["last_seen_ago"] = format_time_ago((millis() - device_info.last_advertisement_time) / 1000);
      doc["manufacturer_data"] = device_info.manufacturer_data;
      doc["decoded_data"] = device_info.decoded_data;

      std::string payload;
      serializeJson(doc, payload);

      mqtt::global_mqtt_client->publish(topic, payload.c_str(), payload.length(), 0, true);
      ESP_LOGV(TAG, "Published generic data for %s to %s: %s", mac_address_str.c_str(), topic.c_str(), payload.c_str());
    }
  }

  // Periodically send discovery messages for all BTHome devices
  if (millis() - last_bthome_discovery_time_ >= BTHOME_DISCOVERY_INTERVAL_MS) {
    last_bthome_discovery_time_ = millis();
    this->send_all_bthome_discovery_messages();
  }
}

bool CustomBLEScanner::parse_device(const esp32_ble_tracker::ESPBTDevice &device) {
  const esphome::esp32_ble::BLEScanResult &esphome_scan_result = device.get_scan_result();
  std::vector<uint8_t> raw_ad_vector;

  if (esphome_scan_result.ble_adv != nullptr && esphome_scan_result.adv_data_len > 0) {
    raw_ad_vector.assign(esphome_scan_result.ble_adv, esphome_scan_result.ble_adv + esphome_scan_result.adv_data_len);
  }
  std::string raw_data_str = format_vector_to_hex(raw_ad_vector);

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
    return parse_bthome_v2_device(device);
  }

  // Handle generic devices
  BLEDeviceInfo info;
  info.last_advertisement_time = millis();
  info.last_rssi = device.get_rssi();
  
  std::string device_name = device.get_name();
  if (!device_name.empty()) {
    info.name = device_name;
  } else {
    info.name = "N/A";
  }
  
  std::string manuf_data_str = "";
  std::string decoded_manuf_data_str = "";
  for (const auto &manuf_data : device.get_manufacturer_datas()) {
    uint16_t company_id = manuf_data.uuid.get_uuid().uuid.uuid16;
    
    char buffer[7];
    snprintf(buffer, sizeof(buffer), "0x%04X", company_id);
    manuf_data_str += buffer;
    manuf_data_str += ": ";
    manuf_data_str += format_hex_pretty(manuf_data.data);
    manuf_data_str += " | ";
    
    std::string decoded_part = decode_manufacturer_data(company_id, manuf_data.data);
    if (!decoded_part.empty()) {
      decoded_manuf_data_str += decoded_part + " | ";
    }
  }
  info.manufacturer_data = manuf_data_str;
  info.decoded_data = decoded_manuf_data_str;
  
  known_ble_devices_[device.address_uint64()] = info;

  return true;
}

void CustomBLEScanner::send_bthome_discovery_messages_for_device(BTHomeDevice *device_obj) {
    if (mqtt::global_mqtt_client == nullptr || !mqtt::global_mqtt_client->is_connected()) {
        ESP_LOGW(TAG, "MQTT not connected, skipping discovery for %s", mac_address_to_string(device_obj->address).c_str());
        return;
    }

    std::string node_name = App.get_name();
    std::string mac_address_clean = mac_address_to_clean_string(device_obj->address);
    std::string ha_device_identifier = node_name + "_" + mac_address_clean;
    std::string ha_device_name = device_obj->get_ha_device_name();

    if (ha_device_name.empty()) {
        ha_device_name = "BTHome " + mac_address_to_string(device_obj->address);
        ESP_LOGW(TAG, "HA device name for %s was empty. Falling back to MAC-based name: '%s'",
                 mac_address_to_string(device_obj->address).c_str(), ha_device_name.c_str());
    }

    ESP_LOGD(TAG, "DEBUG: HA Device Name for discovery: '%s'", ha_device_name.c_str());

    JsonDocument device_doc;
    JsonArray identifiers = device_doc["identifiers"].to<JsonArray>();
    identifiers.add(ha_device_identifier);
    device_doc["name"] = ha_device_name;
    device_doc["mdl"] = "BTHome v2 Device";
    device_doc["mf"] = "ESPHome BLE Gateway";

    std::string base_state_topic = "esphome/" + node_name + "/bthome/" + mac_address_clean + "/state";
    const std::string discovery_prefix = mqtt::global_mqtt_client->get_discovery_info().prefix;

    auto send_sensor_discovery = [&](const std::string& sensor_type, const std::string& unit, const std::string& dev_class, const std::string& state_class) {
        JsonDocument sensor_config_doc;
        sensor_config_doc["name"] = sensor_type;
        std::string entity_id_clean = to_lower_string(sensor_type);
        
        sensor_config_doc["unique_id"] = ha_device_identifier + "_" + entity_id_clean;
        sensor_config_doc["state_topic"] = base_state_topic;
        sensor_config_doc["value_template"] = "{{ value_json." + entity_id_clean + " }}"; 
        sensor_config_doc["unit_of_measurement"] = unit;
        sensor_config_doc["device_class"] = dev_class;
        sensor_config_doc["state_class"] = state_class;
        sensor_config_doc["expire_after"] = 300;
        sensor_config_doc["device"] = device_doc;

        std::string discovery_topic = discovery_prefix + "/sensor/" + ha_device_identifier + "/" + entity_id_clean + "/config";
        std::string payload;
        serializeJson(sensor_config_doc, payload);

        mqtt::global_mqtt_client->publish(discovery_topic, payload.c_str(), payload.length(), 0, true);
        ESP_LOGD(TAG, "Sent %s discovery for %s (HA Device ID: %s)", sensor_type.c_str(), mac_address_to_string(device_obj->address).c_str(), ha_device_identifier.c_str());
    };

    // UPDATED: Dynamically send discovery only for available sensors
    for (const auto& [type_id, measurement] : device_obj->measurements) {
        send_sensor_discovery(measurement.name, measurement.unit, measurement.device_class, measurement.state_class);
    }
}

void CustomBLEScanner::send_all_bthome_discovery_messages() {
  if (mqtt::global_mqtt_client == nullptr || !mqtt::global_mqtt_client->is_connected()) {
    ESP_LOGW(TAG, "MQTT not connected, skipping global BTHome discovery scan.");
    return;
  }

  ESP_LOGD(TAG, "Starting global BTHome discovery refresh for %zu devices.", bthome_devices_.size());
  for (auto const& [mac_address_u64, device_obj] : bthome_devices_) {
    if (millis() - device_obj->last_seen_millis_ < 600000) {
      this->send_bthome_discovery_messages_for_device(device_obj);
    } else {
       ESP_LOGV(TAG, "Skipping discovery for inactive BTHome device %s", mac_address_to_string(device_obj->address).c_str());
    }
  }
}

bool CustomBLEScanner::parse_bthome_v2_device(const esp32_ble_tracker::ESPBTDevice &device) {
  if (mqtt::global_mqtt_client == nullptr || !mqtt::global_mqtt_client->is_connected()) {
    ESP_LOGW(TAG, "MQTT not connected, skipping BTHome parsing for %s", device.address_str().c_str());
    return false;
  }

  const std::vector<uint8_t>* bthome_data_ptr = nullptr;
  for (const auto &entry : device.get_service_datas()) {
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
  if ((flags & 0x01) != 0) {
    ESP_LOGW(TAG, "BTHome v2 device %s is encrypted! Decryption not implemented in this component.", device.address_str().c_str());
    return false;
  }

  uint64_t device_mac_u64 = device.address_uint64();
  BTHomeDevice *bthome_dev;
  auto it = this->bthome_devices_.find(device_mac_u64);

  if (it == this->bthome_devices_.end()) {
    ESP_LOGI(TAG, "First time seeing BTHome v2 device: %s. Creating new BTHomeDevice object.", device.address_str().c_str());
    bthome_dev = new BTHomeDevice(device.address());
    this->bthome_devices_[device_mac_u64] = bthome_dev;
    bthome_dev->update_ha_device_name("BTHome " + mac_address_to_string(bthome_dev->address));
    this->send_bthome_discovery_messages_for_device(bthome_dev);
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
  while (offset < bthome_data.size()) {
    uint8_t type = bthome_data[offset];
    offset++;
    float value = NAN;
    bool parsed_successfully = true;

    switch (type) {
      case BTHOME_PACKET_ID: {
        if (offset + 1 > bthome_data.size()) { parsed_successfully = false; break;}
        ESP_LOGD(TAG, "  BTHome Packet ID: 0x%02X", bthome_data[offset]);
        offset += 1;
        break;
      }
      case BTHOME_MEASUREMENT_TEMPERATURE: {
        if (offset + 2 > bthome_data.size()) { parsed_successfully = false; break;}
        value = (int16_t) (bthome_data[offset] | (bthome_data[offset+1] << 8)) * 0.01f;
        state_doc["temperature"] = round(value * 100) / 100.0;
        offset += 2;
        break;
      }
      case BTHOME_MEASUREMENT_HUMIDITY: {
        if (offset + 2 > bthome_data.size()) { parsed_successfully = false; break;}
        value = (uint16_t) (bthome_data[offset] | (bthome_data[offset+1] << 8)) * 0.01f;
        state_doc["humidity"] = round(value * 100) / 100.0;
        offset += 2;
        break;
      }
      case BTHOME_MEASUREMENT_BATTERY: {
        if (offset + 1 > bthome_data.size()) { parsed_successfully = false; break;}
        value = (float)bthome_data[offset];
        state_doc["battery"] = (int)value;
        offset += 1;
        break;
      }
      case BTHOME_MEASUREMENT_VOLTAGE: {
        if (offset + 2 > bthome_data.size()) { parsed_successfully = false; break;}
        value = (uint16_t) (bthome_data[offset] | (bthome_data[offset+1] << 8)) * 0.001f;
        state_doc["voltage"] = round(value * 1000) / 1000.0;
        offset += 2;
        break;
      }
      case BTHOME_MEASUREMENT_PRESSURE: {
        if (offset + 3 > bthome_data.size()) { parsed_successfully = false; break; }
        value = (float)(bthome_data[offset] | (bthome_data[offset+1] << 8) | (bthome_data[offset+2] << 16)) * 0.01f;
        state_doc["pressure"] = round(value * 100) / 100.0;
        offset += 3;
        break;
      }
      case BTHOME_MEASUREMENT_ILLUMINANCE: {
        if (offset + 3 > bthome_data.size()) { parsed_successfully = false; break; }
        value = (float)(bthome_data[offset] | (bthome_data[offset+1] << 8) | (bthome_data[offset+2] << 16)) * 0.01f;
        state_doc["illuminance"] = round(value * 100) / 100.0;
        offset += 3;
        break;
      }
      case BTHOME_MEASUREMENT_MASS_KG: {
        if (offset + 2 > bthome_data.size()) { parsed_successfully = false; break; }
        value = (float)(bthome_data[offset] | (bthome_data[offset+1] << 8)) * 0.01f;
        state_doc["mass_kg"] = round(value * 100) / 100.0;
        offset += 2;
        break;
      }
      case BTHOME_MEASUREMENT_MASS_LB: {
        if (offset + 2 > bthome_data.size()) { parsed_successfully = false; break; }
        value = (float)(bthome_data[offset] | (bthome_data[offset+1] << 8)) * 0.01f;
        state_doc["mass_lb"] = round(value * 100) / 100.0;
        offset += 2;
        break;
      }
      case BTHOME_MEASUREMENT_DEWPOINT: {
        if (offset + 2 > bthome_data.size()) { parsed_successfully = false; break; }
        value = (int16_t)(bthome_data[offset] | (bthome_data[offset+1] << 8)) * 0.01f;
        state_doc["dewpoint"] = round(value * 100) / 100.0;
        offset += 2;
        break;
      }
      case BTHOME_MEASUREMENT_COUNT_S: {
        if (offset + 1 > bthome_data.size()) { parsed_successfully = false; break; }
        value = (float)bthome_data[offset];
        state_doc["count_s"] = (int)value;
        offset += 1;
        break;
      }
      case BTHOME_MEASUREMENT_COUNT_M: {
        if (offset + 2 > bthome_data.size()) { parsed_successfully = false; break; }
        value = (float)(bthome_data[offset] | (bthome_data[offset+1] << 8));
        state_doc["count_m"] = (int)value;
        offset += 2;
        break;
      }
      case BTHOME_MEASUREMENT_COUNT_L: {
        if (offset + 4 > bthome_data.size()) { parsed_successfully = false; break; }
        value = (float)(bthome_data[offset] | (bthome_data[offset+1] << 8) | (bthome_data[offset+2] << 16) | (bthome_data[offset+3] << 24));
        state_doc["count_l"] = (int)value;
        offset += 4;
        break;
      }
      case BTHOME_MEASUREMENT_ENERGY: {
        if (offset + 3 > bthome_data.size()) { parsed_successfully = false; break; }
        value = (float)(bthome_data[offset] | (bthome_data[offset+1] << 8) | (bthome_data[offset+2] << 16)) * 0.001f;
        state_doc["energy"] = round(value * 1000) / 1000.0;
        offset += 3;
        break;
      }
      case BTHOME_MEASUREMENT_POWER: {
        if (offset + 3 > bthome_data.size()) { parsed_successfully = false; break; }
        value = (float)(bthome_data[offset] | (bthome_data[offset+1] << 8) | (bthome_data[offset+2] << 16)) * 0.01f;
        state_doc["power"] = round(value * 100) / 100.0;
        offset += 3;
        break;
      }
      case BTHOME_MEASUREMENT_CO2: {
        if (offset + 2 > bthome_data.size()) { parsed_successfully = false; break; }
        value = (float)(bthome_data[offset] | (bthome_data[offset+1] << 8));
        state_doc["co2"] = (int)value;
        offset += 2;
        break;
      }
      case BTHOME_MEASUREMENT_VOC: {
        if (offset + 2 > bthome_data.size()) { parsed_successfully = false; break; }
        value = (float)(bthome_data[offset] | (bthome_data[offset+1] << 8));
        state_doc["voc"] = (int)value;
        offset += 2;
        break;
      }
      default: {
        ESP_LOGW(TAG, "Unknown BTHome v2 data type: 0x%02X for %s at offset %zu. Full payload: %s",
                  type, device.address_str().c_str(), offset - 1, format_vector_to_hex(bthome_data).c_str());
        offset = bthome_data.size();
        parsed_successfully = false;
        break;
      }
    }
    if (parsed_successfully && !std::isnan(value)) {
        bthome_dev->measurements[type] = {
            .name = BTHomeDataTypeToString(type),
            .value = value,
            .unit = BTHomeDataTypeToUnit(type),
            .device_class = BTHomeDataTypeToDeviceClass(type),
            .state_class = BTHomeDataTypeToStateClass(type)
        };
    }
  }

  std::string state_topic = "esphome/" + App.get_name() + "/bthome/" + mac_address_to_clean_string(device_mac_u64) + "/state";
  std::string state_payload;
  serializeJson(state_doc, state_payload);
  if (!state_doc.isNull() && state_doc.size() > 0) {
    mqtt::global_mqtt_client->publish(state_topic, state_payload.c_str(), state_payload.length(), 0, false);
    ESP_LOGV(TAG, "Published BTHome state for %s to %s: %s", device.address_str().c_str(), state_topic.c_str(), state_payload.c_str());
  } else {
    ESP_LOGD(TAG, "No BTHome measurements parsed for %s, skipping MQTT state publish.", device.address_str().c_str());
  }

  return true;
}

void CustomBLEScanner::prune_stale_devices() {
  const uint32_t now = millis();
  const uint32_t stale_timeout = 900000; // 15 minutes
  ESP_LOGD(TAG, "Pruning stale devices from memory (older than %u ms)...", stale_timeout);

  int pruned_generic_count = 0;
  for (auto it = known_ble_devices_.begin(); it != known_ble_devices_.end();) {
    if (now - it->second.last_advertisement_time > stale_timeout) {
      it = known_ble_devices_.erase(it);
      pruned_generic_count++;
    } else {
      ++it;
    }
  }
  if (pruned_generic_count > 0) {
    ESP_LOGD(TAG, "Pruned %d stale generic devices from memory.", pruned_generic_count);
  }

  int pruned_bthome_count = 0;
  for (auto it = bthome_devices_.begin(); it != bthome_devices_.end();) {
    if (now - it->second->last_seen_millis_ > stale_timeout) {
      delete it->second;
      it = bthome_devices_.erase(it);
      pruned_bthome_count++;
    } else {
      ++it;
    }
  }
  if (pruned_bthome_count > 0) {
    ESP_LOGD(TAG, "Pruned %d stale BTHome devices from memory.", pruned_bthome_count);
  }
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
  ESP_LOGCONFIG(TAG, "  BTHome devices will be dynamically discovered via MQTT.");
}

void CustomBLEScanner::set_rssi_threshold(int rssi) {
  rssi_threshold_ = rssi;
  ESP_LOGD(TAG, "RSSI threshold set to %d dBm", rssi_threshold_);
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
        case BTHOME_MEASUREMENT_MASS_LB: return "Mass";
        case BTHOME_MEASUREMENT_DEWPOINT: return "Dewpoint";
        case BTHOME_MEASUREMENT_COUNT_S: return "Count";
        case BTHOME_MEASUREMENT_COUNT_M: return "Count";
        case BTHOME_MEASUREMENT_COUNT_L: return "Count";
        case BTHOME_MEASUREMENT_ENERGY: return "Energy";
        case BTHOME_MEASUREMENT_POWER: return "Power";
        case BTHOME_MEASUREMENT_VOLTAGE: return "Voltage";
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
        case BTHOME_MEASUREMENT_CO2: return "carbon_dioxide";
        case BTHOME_MEASUREMENT_VOC: return "volatile_organic_compounds";
        default: return "";
    }
}

std::string BTHomeDataTypeToStateClass(uint8_t type) {
    switch (type) {
        case BTHOME_MEASUREMENT_BATTERY: return "measurement";
        case BTHOME_MEASUREMENT_TEMPERATURE: return "measurement";
        case BTHOME_MEASUREMENT_HUMIDITY: return "measurement";
        case BTHOME_MEASUREMENT_PRESSURE: return "measurement";
        case BTHOME_MEASUREMENT_ILLUMINANCE: return "measurement";
        case BTHOME_MEASUREMENT_MASS_KG: return "measurement";
        case BTHOME_MEASUREMENT_MASS_LB: return "measurement";
        case BTHOME_MEASUREMENT_DEWPOINT: return "measurement";
        case BTHOME_MEASUREMENT_COUNT_S: return "total_increasing";
        case BTHOME_MEASUREMENT_COUNT_M: return "total_increasing";
        case BTHOME_MEASUREMENT_COUNT_L: return "total_increasing";
        case BTHOME_MEASUREMENT_ENERGY: return "total_increasing";
        case BTHOME_MEASUREMENT_POWER: return "measurement";
        case BTHOME_MEASUREMENT_VOLTAGE: return "measurement";
        case BTHOME_MEASUREMENT_CO2: return "measurement";
        case BTHOME_MEASUREMENT_VOC: return "measurement";
        default: return "";
    }
}

} // namespace custom_ble_scanner
} // namespace esphome