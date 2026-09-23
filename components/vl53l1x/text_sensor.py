import esphome.codegen as cg
from esphome.components import text_sensor
import esphome.config_validation as cv
from esphome.const import ENTITY_CATEGORY_DIAGNOSTIC, ICON_CHECK_CIRCLE_OUTLINE

from . import CONF_VL53L1X_ID, VL53L1XComponent

DEPENDENCIES = ["vl53l1x"]

CONF_RANGE_STATUS = "range_status"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_VL53L1X_ID): cv.use_id(VL53L1XComponent),
        cv.Optional(CONF_RANGE_STATUS): text_sensor.text_sensor_schema(
            icon=ICON_CHECK_CIRCLE_OUTLINE,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_VL53L1X_ID])
    if range_status_config := config.get(CONF_RANGE_STATUS):
        sens = await text_sensor.new_text_sensor(range_status_config)
        cg.add(hub.set_range_status_text_sensor(sens))
