import esphome.codegen as cg
from esphome.components import switch
import esphome.config_validation as cv
from esphome.const import CONF_UPDATE_INTERVAL

from .. import CONF_BUS_T4_ID, BusT4Component, bus_t4_ns

DEPENDENCIES = ['bus_t4']

BusT4ConfigSwitch = bus_t4_ns.class_('BusT4ConfigSwitch', switch.Switch, cg.Component)

CONF_SETTING = 'setting'

# On/off registers, per gashtaan/nice-bidiwifi-firmware's t4.h
SETTINGS = {
    'auto_close': 0x80,
    'photo_close': 0x84,
    'always_close': 0x88,
    'standby': 0x8C,
    'peak': 0x90,
    'pre_flash': 0x94,
}

CONFIG_SCHEMA = (
    # DISABLED: restoring a state on boot would write the controller on every restart
    switch.switch_schema(BusT4ConfigSwitch, default_restore_mode='DISABLED')
    .extend(
        {
            cv.GenerateID(CONF_BUS_T4_ID): cv.use_id(BusT4Component),
            cv.Required(CONF_SETTING): cv.one_of(*SETTINGS, lower=True),
            cv.Optional(CONF_UPDATE_INTERVAL, default='60s'): cv.positive_time_period_milliseconds,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = await switch.new_switch(config)
    await cg.register_component(var, config)

    bus = await cg.get_variable(config[CONF_BUS_T4_ID])
    cg.add(var.set_parent(bus))
    cg.add(var.set_register(SETTINGS[config[CONF_SETTING]]))
