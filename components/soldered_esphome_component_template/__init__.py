import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

CODEOWNERS = ["@SolderedElectronics"]

soldered_esphome_component_template_ns = cg.esphome_ns.namespace("soldered_esphome_component_template")
SolderedEsphomeComponentTemplate = soldered_esphome_component_template_ns.class_(
    "SolderedEsphomeComponentTemplate", cg.Component
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(SolderedEsphomeComponentTemplate),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
