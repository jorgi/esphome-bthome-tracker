# ESPHome BTHome BLE Tracker

This custom ESPHome component tracks BTHome BLE sensors (e.g., Tuya with PVVX firmware )
and publishes MQTT discovery messages for Home Assistant.

## Features

- Parses BLE advertisements for temperature, humidity, voltage, and RSSI.
- Publishes Home Assistant MQTT discovery topics.
- Auto-discovers up to 10 BTHome devices.
- Reuses ESP32 BLE Tracker and MQTT from ESPHome.

## Installation

1. Add to your ESPHome config:

```yaml
external_components:
  - source: github://yourusername/esphome-bthome-tracker
    components: [bthome_tracker]
