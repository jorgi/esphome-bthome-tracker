// custom_components/custom_ble_scanner/custom_ble_scanner.h
#pragma once

#include "esphome/components/esp32_ble_tracker/esp32_ble_tracker.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/mqtt/mqtt_client.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"

#include <map>
#include <vector>
#include <string>
#include <set>
#include <algorithm>
#include <cctype>
#include <cmath>

namespace esphome {
namespace custom_ble_scanner {

// --- Helper function declarations ---
std::string to_lower_string(std::string s);
std::string mac_address_to_string(uint64_t mac);
std::string mac_address_to_string(const uint8_t* mac_array);
std::string mac_address_to_clean_string(uint64_t mac);
std::string mac_address_to_clean_string(const uint8_t* mac_array);
// --- End Helper function declarations ---


const esp32_ble_tracker::ESPBTUUID BTHOME_V2_SERVICE_UUID = esp32_ble_tracker::ESPBTUUID::from_uint16(0xFCD2);

// BTHome Data Types
enum BTHomeDataType {
  BTHOME_PACKET_ID = 0x00,
  BTHOME_MEASUREMENT_BATTERY = 0x01,
  BTHOME_MEASUREMENT_TEMPERATURE = 0x02,
  BTHOME_MEASUREMENT_HUMIDITY = 0x03,
  BTHOME_MEASUREMENT_PRESSURE = 0x04,
  BTHOME_MEASUREMENT_ILLUMINANCE = 0x05,
  BTHOME_MEASUREMENT_MASS_KG = 0x06,
  BTHOME_MEASUREMENT_MASS_LB = 0x07,
  BTHOME_MEASUREMENT_DEWPOINT = 0x08,
  BTHOME_MEASUREMENT_COUNT_S = 0x09,
  BTHOME_MEASUREMENT_ENERGY = 0x0A,
  BTHOME_MEASUREMENT_POWER = 0x0B,
  BTHOME_MEASUREMENT_VOLTAGE = 0x0C,
  BTHOME_MEASUREMENT_DISTANCE = 0x10,      // RE-ADDED
  BTHOME_MEASUREMENT_CO2 = 0x12,
  BTHOME_MEASUREMENT_VOC = 0x13,
  BTHOME_MEASUREMENT_COUNT_LEGACY = 0x15, // RE-ADDED
  BTHOME_MEASUREMENT_COUNT_M = 0x3D,
  BTHOME_MEASUREMENT_COUNT_L = 0x3E,
};

// Struct to hold a single BTHome measurement
struct BTHomeMeasurement {
  std::string name;
  float value;
  std::string unit;
  std::string device_class;
  std::string state_class;
};

// Class to represent a single BTHome device discovered
class BTHomeDevice {
 public:
  uint8_t address[6];
  std::map<uint8_t, BTHomeMeasurement> measurements;
  std::string current_ha_name_;
  uint32_t last_seen_millis_;

  BTHomeDevice(const uint8_t *addr) : last_seen_millis_(0) {
    memcpy(address, addr, 6);
    this->current_ha_name_ = "";
  }

  const std::string& get_ha_device_name() const {
    return this->current_ha_name_;
  }

  void update_ha_device_name(const std::string& new_name) {
    std::string lower_new_name = to_lower_string(new_name);
    if (!lower_new_name.empty() && lower_new_name != "nan" && this->current_ha_name_ != new_name) {
      this->current_ha_name_ = new_name;
      ESP_LOGD("BTHomeDevice", "Updated HA device name for %s to: '%s'",
               mac_address_to_string(this->address).c_str(), new_name.c_str());
    }
  }
};


struct BLEDeviceInfo {
  uint32_t last_advertisement_time;
  int last_rssi;
  std::string name;
  std::string manufacturer_data;
  std::map<std::string, std::string> decoded_info;
};

class CustomBLEScanner : public esphome::Component, public esphome::esp32_ble_tracker::ESPBTDeviceListener {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  bool parse_device(const esp32_ble_tracker::ESPBTDevice &device) override;

  void send_bthome_discovery_messages_for_device(BTHomeDevice *device_obj);
  void send_all_bthome_discovery_messages();

  bool parse_bthome_v2_device(const esp32_ble_tracker::ESPBTDevice &device);

  void set_esp32_ble_tracker(esp32_ble_tracker::ESP32BLETracker *tracker) { tracker_ = tracker; }
  void set_rssi_threshold(int rssi) { rssi_threshold_ = rssi; }
  void set_generic_publish_interval(uint32_t interval_ms) { generic_publish_interval_ms_ = interval_ms; }
  void set_ble_raw_data_text_sensor(text_sensor::TextSensor *sensor) { ble_raw_data_sensor_ = sensor; }
  void set_prune_timeout(uint32_t timeout) { prune_timeout_ = timeout; }
  void set_discovery_interval(uint32_t interval) { bthome_discovery_interval_ms_ = interval; }
  void set_generic_scanner_enabled(bool enabled) { generic_scanner_enabled_ = enabled; }

 protected:
  void prune_stale_devices();
  esp32_ble_tracker::ESP32BLETracker *tracker_{nullptr};
  int rssi_threshold_{-100};
  uint32_t last_log_time_{0};
  uint32_t last_prune_time_{0};
  
  bool generic_scanner_enabled_{true};
  uint32_t generic_publish_interval_ms_{60000};
  uint32_t prune_timeout_{900000};
  uint32_t bthome_discovery_interval_ms_{300000};

  std::map<uint64_t, BLEDeviceInfo> known_ble_devices_;
  std::map<uint64_t, BTHomeDevice*> bthome_devices_;
  uint32_t last_generic_publish_time_{0};
  uint32_t last_bthome_discovery_time_{0};

  text_sensor::TextSensor *ble_raw_data_sensor_{nullptr};
};

// Declare helper functions
std::string BTHomeDataTypeToString(uint8_t type);
std::string BTHomeDataTypeToUnit(uint8_t type);
std::string BTHomeDataTypeToDeviceClass(uint8_t type);
std::string BTHomeDataTypeToStateClass(uint8_t type);

} // namespace custom_ble_scanner
} // namespace esphome