#pragma once

#include "esphome/core/component.h"
#include "esphome/components/esp32_ble_tracker/esp32_ble_tracker.h"
#include "esphome/components/mqtt/mqtt_client.h"

namespace esphome {
namespace bthome_tracker {

class BTHomeTracker : public sensor::Sensor, public Component {
 public:
  void setup() override;
  void loop() override;
};


struct BTHomeDevice {
  std::string mac;
  std::string name;
  int rssi;
  float temperature;
  float humidity;
  float voltage;
  bool valid = false;
  bool discovered = false;
};

class BTHomeTracker : public Component, public esp32_ble_tracker::ESPBTDeviceListener {
 public:
  void on_ble_advertise(const esp32_ble_tracker::ESPBTDevice &device) override;
  void setup() override {}
  void loop() override;
  void set_mqtt_client(mqtt::MQTTClientComponent *mqtt_client) { mqtt_client_ = mqtt_client; }

 protected:
  mqtt::MQTTClientComponent *mqtt_client_;
  BTHomeDevice devices_[10];
  int device_count_ = 0;
  unsigned long last_publish_ = 0;

  void publish_discovery(int index);
  void publish_state(int index);
};

}  // namespace bthome_tracker
}  // namespace esphome
