import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_DISTANCE,
    DEVICE_CLASS_DISTANCE,
    ENTITY_CATEGORY_DIAGNOSTIC,
    ICON_BRIGHTNESS_5,
    ICON_SIGNAL,
    STATE_CLASS_MEASUREMENT,
    UNIT_METER,
)

from . import CONF_VL53L1X_ID, VL53L1XComponent

DEPENDENCIES = ["vl53l1x"]

CONF_SIGNAL_RATE = "signal_rate"
CONF_AMBIENT_RATE = "ambient_rate"
UNIT_MCPS = "Mcps"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_VL53L1X_ID): cv.use_id(VL53L1XComponent),
        cv.Optional(CONF_DISTANCE): sensor.sensor_schema(
            unit_of_measurement=UNIT_METER,
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_DISTANCE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_SIGNAL_RATE): sensor.sensor_schema(
            unit_of_measurement=UNIT_MCPS,
            icon=ICON_SIGNAL,
            accuracy_decimals=2,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_AMBIENT_RATE): sensor.sensor_schema(
            unit_of_measurement=UNIT_MCPS,
            icon=ICON_BRIGHTNESS_5,
            accuracy_decimals=2,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_VL53L1X_ID])
    if distance_config := config.get(CONF_DISTANCE):
        sens = await sensor.new_sensor(distance_config)
        cg.add(hub.set_distance_sensor(sens))
    if signal_rate_config := config.get(CONF_SIGNAL_RATE):
        sens = await sensor.new_sensor(signal_rate_config)
        cg.add(hub.set_signal_rate_sensor(sens))
    if ambient_rate_config := config.get(CONF_AMBIENT_RATE):
        sens = await sensor.new_sensor(ambient_rate_config)
        cg.add(hub.set_ambient_rate_sensor(sens))
