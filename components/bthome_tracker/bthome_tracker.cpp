#include "bthome_tracker.h"
#include "esphome/core/log.h"

namespace esphome {
namespace bthome_tracker {

static const char *const TAG = "bthome_tracker";

void BTHomeTracker::on_ble_advertise(const esp32_ble_tracker::ESPBTDevice &x) {
  auto mac = x.address_str();
  mac.erase(std::remove(mac.begin(), mac.end(), ':'), mac.end());
  auto name = x.get_name();
  int rssi = x.get_rssi();

  auto service_datas = x.get_service_datas();
  const std::vector<uint8_t> *service_data = nullptr;
  for (const auto &data : service_datas) {
    if (data.uuid == esp32_ble_tracker::ESPBTUUID::from_uint16(0xFCD2)) {
      service_data = &data.data;
      break;
    }
  }
  if (service_data == nullptr || service_data->size() < 14) {
    return;
  }

  float temp = ((*service_data)[6] | ((*service_data)[7] << 8)) / 100.0;
  float hum = ((*service_data)[9] | ((*service_data)[10] << 8)) / 100.0;
  float volt = ((*service_data)[12] | ((*service_data)[13] << 8)) / 1000.0;

  if ((*service_data)[5] != 0x02 || (*service_data)[8] != 0x03 || (*service_data)[11] != 0x0C ||
      temp <= -40 || temp >= 100 || hum < 0 || hum > 100 || volt <= 0 || volt >= 4) {
    return;
  }

  int index = -1;
  for (int i = 0; i < device_count_; i++) {
    if (devices_[i].mac == mac) {
      index = i;
      break;
    }
  }

  if (index == -1 && device_count_ < 10) {
    index = device_count_++;
    devices_[index].mac = mac;
  }

  if (index != -1) {
    auto &d = devices_[index];
    d.name = name;
    d.rssi = rssi;
    d.temperature = temp;
    d.humidity = hum;
    d.voltage = volt;
    d.valid = true;
  }
}

void BTHomeTracker::loop() {
  const auto now = millis();
  if (now - last_publish_ < 60000) return;
  last_publish_ = now;

  for (int i = 0; i < device_count_; i++) {
    if (!devices_[i].valid) continue;
    if (!devices_[i].discovered) {
      publish_discovery(i);
      devices_[i].discovered = true;
    }
    publish_state(i);
  }
}

void BTHomeTracker::publish_discovery(int i) {
  auto &d = devices_[i];
  std::string id_base = "bthome_" + d.mac;

  mqtt_client_->publish_json("homeassistant/sensor/" + id_base + "_temperature/config", [=](JsonObject root) {
    root["name"] = id_base + " Temperature";
    root["state_topic"] = "homeassistant/sensor/" + id_base + "_temperature/state";
    root["unit_of_measurement"] = "°C";
    root["device_class"] = "temperature";
    root["unique_id"] = id_base + "_temperature";
  });

  mqtt_client_->publish_json("homeassistant/sensor/" + id_base + "_humidity/config", [=](JsonObject root) {
    root["name"] = id_base + " Humidity";
    root["state_topic"] = "homeassistant/sensor/" + id_base + "_humidity/state";
    root["unit_of_measurement"] = "%";
    root["device_class"] = "humidity";
    root["unique_id"] = id_base + "_humidity";
  });

  mqtt_client_->publish_json("homeassistant/sensor/" + id_base + "_voltage/config", [=](JsonObject root) {
    root["name"] = id_base + " Voltage";
    root["state_topic"] = "homeassistant/sensor/" + id_base + "_voltage/state";
    root["unit_of_measurement"] = "V";
    root["device_class"] = "voltage";
    root["unique_id"] = id_base + "_voltage";
  });

  mqtt_client_->publish_json("homeassistant/sensor/" + id_base + "_rssi/config", [=](JsonObject root) {
    root["name"] = id_base + " RSSI";
    root["state_topic"] = "homeassistant/sensor/" + id_base + "_rssi/state";
    root["unit_of_measurement"] = "dBm";
    root["device_class"] = "signal_strength";
    root["unique_id"] = id_base + "_rssi";
  });

  mqtt_client_->publish_json("homeassistant/text_sensor/" + id_base + "_name/config", [=](JsonObject root) {
    root["name"] = id_base + " Name";
    root["state_topic"] = "homeassistant/text_sensor/" + id_base + "_name/state";
    root["unique_id"] = id_base + "_name";
  });
}

void BTHomeTracker::publish_state(int i) {
  auto &d = devices_[i];
  std::string id_base = "bthome_" + d.mac.substr(0, 6);
  mqtt_client_->publish("homeassistant/sensor/" + id_base + "_temperature/state", to_string(d.temperature));
  mqtt_client_->publish("homeassistant/sensor/" + id_base + "_humidity/state", to_string(d.humidity));
  mqtt_client_->publish("homeassistant/sensor/" + id_base + "_voltage/state", to_string(d.voltage));
  mqtt_client_->publish("homeassistant/sensor/" + id_base + "_rssi/state", to_string(d.rssi));
  mqtt_client_->publish("homeassistant/text_sensor/" + id_base + "_name/state", d.name.empty() ? "Unknown" : d.name);
}

}  // namespace bthome_tracker
}  // namespace esphome
