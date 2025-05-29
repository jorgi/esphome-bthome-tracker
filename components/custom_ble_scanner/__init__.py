import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import esp32_ble_tracker, sensor, text_sensor
from esphome.const import CONF_ID # Keep CONF_ID, it's usually always there

# Define the namespace for our custom component (matches namespace in .h/.cpp)
custom_ble_scanner_ns = cg.esphome_ns.namespace("custom_ble_scanner")

# Declare our CustomBLEScanner C++ class
CustomBLEScanner = custom_ble_scanner_ns.class_(
    "CustomBLEScanner", cg.Component
)

# Define configuration keys for our component
# Changed CONF_RSSI to CONF_RSSI_THRESHOLD to reflect the C++ setter name
CONF_RSSI_THRESHOLD = "rssi_threshold" # Use this name for the YAML key
CONF_ESP32_BLE_ID = "esp32_ble_id"
CONF_GENERIC_PUBLISH_INTERVAL = "generic_publish_interval"
CONF_BTHOME_TEMPERATURE_ID = "bthome_temperature_id"
CONF_BTHOME_HUMIDITY_ID = "bthome_humidity_id"
CONF_BTHOME_BATTERY_ID = "bthome_battery_id"
CONF_BLE_RAW_DATA_TEXT_SENSOR = "ble_raw_data_text_sensor"


# Define the YAML configuration schema for the custom_ble_scanner component
CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(CustomBLEScanner),  # ID for our component
        cv.Required(CONF_ESP32_BLE_ID): cv.use_id(esp32_ble_tracker.ESP32BLETracker), # Link to the esp32_ble_tracker instance
        cv.Optional(CONF_RSSI_THRESHOLD, default=-100): cv.All(cv.int_, cv.Range(min=-120, max=0)), # RSSI threshold
        cv.Optional(CONF_GENERIC_PUBLISH_INTERVAL, default="30s"): cv.positive_time_period_milliseconds, # Interval for generic MQTT
        cv.Optional(CONF_BTHOME_TEMPERATURE_ID): cv.use_id(sensor.Sensor), # Link to temperature sensor
        cv.Optional(CONF_BTHOME_HUMIDITY_ID): cv.use_id(sensor.Sensor),   # Link to humidity sensor
        cv.Optional(CONF_BTHOME_BATTERY_ID): cv.use_id(sensor.Sensor),     # Link to battery sensor
        cv.Optional(CONF_BLE_RAW_DATA_TEXT_SENSOR): cv.use_id(text_sensor.TextSensor), # Link to raw data text sensor
    }
).extend(cv.COMPONENT_SCHEMA) # Inherit from base component schema for 'update_interval' etc.


# 'to_code' function generates the C++ code from the YAML configuration
async def to_code(config):
    # Get the ID of our custom component and create a Pvariable instance
    var = cg.new_Pvariable(config[CONF_ID])

    # Add the component to the global list of components
    await cg.register_component(var, config)

    # Link to the ESP32 BLE Tracker instance
    parent_tracker = await cg.get_variable(config[CONF_ESP32_BLE_ID])
    cg.add(var.set_esp32_ble_tracker(parent_tracker))

    # Set RSSI threshold
    cg.add(var.set_rssi_threshold(config[CONF_RSSI_THRESHOLD])) # Use the new key here

    # Set generic MQTT publish interval
    cg.add(var.set_generic_publish_interval(config[CONF_GENERIC_PUBLISH_INTERVAL]))

    # Link to BTHome sensors if configured
    if CONF_BTHOME_TEMPERATURE_ID in config:
        sens = await cg.get_variable(config[CONF_BTHOME_TEMPERATURE_ID])
        cg.add(var.set_bthome_temperature_sensor(sens))

    if CONF_BTHOME_HUMIDITY_ID in config:
        sens = await cg.get_variable(config[CONF_BTHOME_HUMIDITY_ID])
        cg.add(var.set_bthome_humidity_sensor(sens))

    if CONF_BTHOME_BATTERY_ID in config:
        sens = await cg.get_variable(config[CONF_BTHOME_BATTERY_ID])
        cg.add(var.set_bthome_battery_sensor(sens))
        
    if CONF_BLE_RAW_DATA_TEXT_SENSOR in config:
        text_sens = await cg.get_variable(config[CONF_BLE_RAW_DATA_TEXT_SENSOR])
        cg.add(var.set_ble_raw_data_text_sensor(text_sens))