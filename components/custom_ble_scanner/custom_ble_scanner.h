// custom_components/custom_ble_scanner/custom_ble_scanner.h
#pragma once

#include "esphome/components/esp32_ble_tracker/esp32_ble_tracker.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/mqtt/mqtt_client.h" // For mqtt::MQTTClient
#include "esphome/core/component.h"
#include "esphome/core/helpers.h" // For esphome::format_hex_pretty
#include "esphome/core/application.h" // Needed for App.get_name()
#include "esphome/core/log.h" // For ESP_LOGD

#include <map>
#include <vector>
#include <string>
#include <set>
#include <algorithm> // For std::transform
#include <cctype>    // For std::tolower
#include <cmath>     // For round()

namespace esphome {
namespace custom_ble_scanner {

// --- Helper function declarations (moved to top of namespace) ---
std::string to_lower_string(std::string s);
std::string mac_address_to_string(uint64_t mac);
std::string mac_address_to_string(const uint8_t* mac_array); // Overload
std::string mac_address_to_clean_string(uint64_t mac);
std::string mac_address_to_clean_string(const uint8_t* mac_array); // Overload
// --- End Helper function declarations ---


const esp32_ble_tracker::ESPBTUUID BTHOME_V2_SERVICE_UUID = esp32_ble_tracker::ESPBTUUID::from_uint16(0xFCD2);

// BTHome Data Types
enum BTHomeDataType {
  BTHOME_PACKET_ID = 0x00,
  BTHOME_MEASUREMENT_BATTERY = 0x01,
  BTHOME_MEASUREMENT_TEMPERATURE = 0x02,
  BTHOME_MEASUREMENT_HUMIDITY = 0x03,
  BTHOME_MEASUREMENT_VOLTAGE = 0x0C,
  // Add other BTHome data types as needed
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
  uint8_t address[6]; // Device's MAC address
  std::map<uint8_t, BTHomeMeasurement> measurements; // Store the latest measurements by type ID

  std::string current_ha_name_;
  uint32_t last_seen_millis_;

  // Constructor to initialize with MAC address
  BTHomeDevice(const uint8_t *addr) : last_seen_millis_(0) {
    memcpy(address, addr, 6);
    this->current_ha_name_ = ""; // Will be initialized with MAC-based name on first detection
  }

  // Method to get the current name for Home Assistant
  const std::string& get_ha_device_name() const {
    return this->current_ha_name_;
  }

  // Method to update the Home Assistant device name
  void update_ha_device_name(const std::string& new_name) {
    // Only update if the new name is not empty, not "nan", and is different
    // Use the declared to_lower_string helper
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
  void set_rssi_threshold(int rssi);
  void set_generic_publish_interval(uint32_t interval_ms) { generic_publish_interval_ms_ = interval_ms; }
  void set_ble_raw_data_text_sensor(text_sensor::TextSensor *sensor) { ble_raw_data_sensor_ = sensor; }

  // BTHome sensor setters
  void set_bthome_temperature_sensor(sensor::Sensor *sens) { bthome_temperature_sensor_ = sens; }
  void set_bthome_humidity_sensor(sensor::Sensor *sens) { bthome_humidity_sensor_ = sens; }
  void set_bthome_battery_sensor(sensor::Sensor *sens) { bthome_battery_sensor_ = sens; }
  void set_bthome_voltage_sensor(sensor::Sensor *sens) { bthome_voltage_sensor_ = sens; }

 protected:
  esp32_ble_tracker::ESP32BLETracker *tracker_{nullptr};
  int rssi_threshold_{-100};
  uint32_t last_log_time_{0};
  uint32_t generic_publish_interval_ms_{60000};
  uint32_t last_generic_publish_time_{0};

  std::map<uint64_t, BLEDeviceInfo> known_ble_devices_;

  std::map<uint64_t, BTHomeDevice*> bthome_devices_;
  uint32_t last_bthome_discovery_time_ = 0;
  const uint32_t BTHOME_DISCOVERY_INTERVAL_MS = 300000; // 5 minutes

  text_sensor::TextSensor *ble_raw_data_sensor_{nullptr};

  // BTHome sensor pointers
  sensor::Sensor *bthome_temperature_sensor_{nullptr};
  sensor::Sensor *bthome_humidity_sensor_{nullptr};
  sensor::Sensor *bthome_battery_sensor_{nullptr};
  sensor::Sensor *bthome_voltage_sensor_{nullptr};
};

// Declare helper functions used across BTHomeDevice and CustomBLEScanner
// These are defined in the .cpp file, but declared here for visibility.
std::string BTHomeDataTypeToString(uint8_t type);
std::string BTHomeDataTypeToUnit(uint8_t type);
std::string BTHomeDataTypeToDeviceClass(uint8_t type);
std::string BTHomeDataTypeToStateClass(uint8_t type);


} // namespace custom_ble_scanner
} // namespace esphome