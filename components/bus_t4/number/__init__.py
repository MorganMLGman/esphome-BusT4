import esphome.codegen as cg
from esphome.components import number
import esphome.config_validation as cv
from esphome.const import (
    CONF_MAX_VALUE,
    CONF_MIN_VALUE,
    CONF_STEP,
    CONF_UPDATE_INTERVAL,
    UNIT_PERCENT,
    UNIT_SECOND,
)

from .. import CONF_BUS_T4_ID, BusT4Component, bus_t4_ns

DEPENDENCIES = ['bus_t4']

BusT4ConfigNumber = bus_t4_ns.class_('BusT4ConfigNumber', number.Number, cg.Component)

CONF_SETTING = 'setting'

# Numeric registers, per gashtaan/nice-bidiwifi-firmware's t4.h: (register, min, max, unit)
SETTINGS = {
    'opening_speed': (0x42, 1, 100, UNIT_PERCENT),
    'closing_speed': (0x43, 1, 100, UNIT_PERCENT),
    'pause_time': (0x81, 0, 250, UNIT_SECOND),
    'photo_close_time': (0x85, 0, 250, UNIT_SECOND),
    'always_close_time': (0x89, 0, 250, UNIT_SECOND),
}


def _setting_defaults(config):
    # The unit comes from the setting unless the user picked one
    _reg, _min, _max, unit = SETTINGS[config[CONF_SETTING]]
    config.setdefault('unit_of_measurement', unit)
    return config


CONFIG_SCHEMA = cv.All(
    number.number_schema(BusT4ConfigNumber)
    .extend(
        {
            cv.GenerateID(CONF_BUS_T4_ID): cv.use_id(BusT4Component),
            cv.Required(CONF_SETTING): cv.one_of(*SETTINGS, lower=True),
            cv.Optional(CONF_MIN_VALUE): cv.float_,
            cv.Optional(CONF_MAX_VALUE): cv.float_,
            cv.Optional(CONF_STEP, default=1): cv.positive_float,
            cv.Optional(CONF_UPDATE_INTERVAL, default='60s'): cv.positive_time_period_milliseconds,
        }
    )
    .extend(cv.COMPONENT_SCHEMA),
    _setting_defaults,
)


async def to_code(config):
    register, min_value, max_value, _unit = SETTINGS[config[CONF_SETTING]]
    var = await number.new_number(
        config,
        min_value=config.get(CONF_MIN_VALUE, min_value),
        max_value=config.get(CONF_MAX_VALUE, max_value),
        step=config[CONF_STEP],
    )
    await cg.register_component(var, config)

    bus = await cg.get_variable(config[CONF_BUS_T4_ID])
    cg.add(var.set_parent(bus))
    cg.add(var.set_register(register))
