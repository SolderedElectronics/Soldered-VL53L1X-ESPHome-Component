from esphome import pins
import esphome.codegen as cg
from esphome.components import i2c
import esphome.config_validation as cv
from esphome.const import (
    CONF_ADDRESS,
    CONF_ENABLE_PIN,
    CONF_HEIGHT,
    CONF_ID,
    CONF_TIMEOUT,
    CONF_WIDTH,
)

CODEOWNERS = ["@SolderedElectronics"]
DEPENDENCIES = ["i2c"]
MULTI_CONF = True

CONF_VL53L1X_ID = "vl53l1x_id"
CONF_DISTANCE_MODE = "distance_mode"
CONF_TIMING_BUDGET = "timing_budget"
CONF_ROI = "roi"
CONF_CENTER = "center"

VL53L1X_DEFAULT_ADDRESS = 0x29

vl53l1x_ns = cg.esphome_ns.namespace("vl53l1x")
VL53L1XComponent = vl53l1x_ns.class_(
    "VL53L1XComponent", cg.PollingComponent, i2c.I2CDevice
)

DistanceMode = vl53l1x_ns.enum("DistanceMode")
DISTANCE_MODES = {
    "short": DistanceMode.DISTANCE_MODE_SHORT,
    "medium": DistanceMode.DISTANCE_MODE_MEDIUM,
    "long": DistanceMode.DISTANCE_MODE_LONG,
}


def _validate(config):
    if config[CONF_ADDRESS] != VL53L1X_DEFAULT_ADDRESS and CONF_ENABLE_PIN not in config:
        raise cv.Invalid(
            "An address other than 0x29 requires 'enable_pin' (XSHUT), the sensor forgets its address on power-up "
            "and has to be reset to be moved to it again"
        )
    # Minimum timing budgets from the VL53L1X datasheet
    minimum_us = 20000 if config[CONF_DISTANCE_MODE] == "short" else 33000
    if config[CONF_TIMING_BUDGET].total_microseconds < minimum_us:
        raise cv.Invalid(
            f"'timing_budget' must be at least {minimum_us // 1000}ms in {config[CONF_DISTANCE_MODE]} distance mode"
        )
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(VL53L1XComponent),
            cv.Optional(CONF_ENABLE_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_DISTANCE_MODE, default="long"): cv.enum(
                DISTANCE_MODES, lower=True
            ),
            cv.Optional(CONF_TIMING_BUDGET, default="50ms"): cv.All(
                cv.positive_time_period_microseconds,
                cv.Range(
                    min=cv.TimePeriod(milliseconds=20),
                    max=cv.TimePeriod(milliseconds=1000),
                ),
            ),
            cv.Optional(CONF_TIMEOUT, default="500ms"): cv.All(
                cv.positive_time_period_milliseconds,
                cv.Range(
                    min=cv.TimePeriod(milliseconds=1),
                    max=cv.TimePeriod(seconds=60),
                ),
            ),
            cv.Optional(CONF_ROI): cv.Schema(
                {
                    cv.Optional(CONF_WIDTH, default=16): cv.int_range(min=4, max=16),
                    cv.Optional(CONF_HEIGHT, default=16): cv.int_range(min=4, max=16),
                    cv.Optional(CONF_CENTER): cv.int_range(min=0, max=255),
                }
            ),
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(i2c.i2c_device_schema(VL53L1X_DEFAULT_ADDRESS)),
    _validate,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    if CONF_ENABLE_PIN in config:
        enable_pin = await cg.gpio_pin_expression(config[CONF_ENABLE_PIN])
        cg.add(var.set_enable_pin(enable_pin))
    cg.add(var.set_distance_mode(config[CONF_DISTANCE_MODE]))
    cg.add(var.set_timing_budget_us(config[CONF_TIMING_BUDGET]))
    cg.add(var.set_timeout_ms(config[CONF_TIMEOUT]))
    if roi := config.get(CONF_ROI):
        cg.add(var.set_roi_size(roi[CONF_WIDTH], roi[CONF_HEIGHT]))
        if CONF_CENTER in roi:
            cg.add(var.set_roi_center(roi[CONF_CENTER]))
