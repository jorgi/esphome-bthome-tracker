import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import esp32_ble_tracker, text_sensor
from esphome.const import CONF_ID

# Define the namespace for our custom component
custom_ble_scanner_ns = cg.esphome_ns.namespace("custom_ble_scanner")

# Declare our CustomBLEScanner C++ class
CustomBLEScanner = custom_ble_scanner_ns.class_(
    "CustomBLEScanner", cg.Component
)

# Define configuration keys for our component
CONF_RSSI_THRESHOLD = "rssi_threshold"
CONF_ESP32_BLE_ID = "esp32_ble_id"
CONF_GENERIC_PUBLISH_INTERVAL = "generic_publish_interval"
CONF_BLE_RAW_DATA_TEXT_SENSOR = "ble_raw_data_text_sensor"
CONF_PRUNE_TIMEOUT = "prune_timeout"
CONF_DISCOVERY_INTERVAL = "bthome_discovery_interval"
CONF_ENABLE_GENERIC_SCANNER = "enable_generic_scanner"
CONF_IGNORE_MAC_ADDRESSES = "ignore_mac_addresses" # NEW

# Define the YAML configuration schema
CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(CustomBLEScanner),
        cv.Required(CONF_ESP32_BLE_ID): cv.use_id(esp32_ble_tracker.ESP32BLETracker),
        cv.Optional(CONF_RSSI_THRESHOLD, default=-100): cv.All(cv.int_, cv.Range(min=-120, max=0)),
        cv.Optional(CONF_GENERIC_PUBLISH_INTERVAL, default="30s"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_BLE_RAW_DATA_TEXT_SENSOR): cv.use_id(text_sensor.TextSensor),
        cv.Optional(CONF_PRUNE_TIMEOUT, default="15min"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_DISCOVERY_INTERVAL, default="5min"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_ENABLE_GENERIC_SCANNER, default=True): cv.boolean,
        cv.Optional(CONF_IGNORE_MAC_ADDRESSES): cv.All([cv.mac_address]), # NEW
    }
).extend(cv.COMPONENT_SCHEMA)

# 'to_code' function generates the C++ code from the YAML configuration
async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    parent_tracker = await cg.get_variable(config[CONF_ESP32_BLE_ID])
    cg.add(var.set_esp32_ble_tracker(parent_tracker))
    cg.add(var.set_rssi_threshold(config[CONF_RSSI_THRESHOLD]))
    cg.add(var.set_generic_publish_interval(config[CONF_GENERIC_PUBLISH_INTERVAL]))
    cg.add(var.set_prune_timeout(config[CONF_PRUNE_TIMEOUT]))
    cg.add(var.set_discovery_interval(config[CONF_DISCOVERY_INTERVAL]))
    cg.add(var.set_generic_scanner_enabled(config[CONF_ENABLE_GENERIC_SCANNER]))

    if CONF_BLE_RAW_DATA_TEXT_SENSOR in config:
        text_sens = await cg.get_variable(config[CONF_BLE_RAW_DATA_TEXT_SENSOR])
        cg.add(var.set_ble_raw_data_text_sensor(text_sens))
    
    # NEW: Pass the list of MAC addresses to the C++ code
    if CONF_IGNORE_MAC_ADDRESSES in config:
        macs = [str(mac) for mac in config[CONF_IGNORE_MAC_ADDRESSES]]
        cg.add(var.set_ignore_mac_addresses(macs))