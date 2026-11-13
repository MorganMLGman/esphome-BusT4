import esphome.codegen as cg
from esphome.components import binary_sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_UPDATE_INTERVAL,
    DEVICE_CLASS_BATTERY,
    DEVICE_CLASS_HEAT,
    DEVICE_CLASS_LOCK,
    DEVICE_CLASS_PROBLEM,
    DEVICE_CLASS_RUNNING,
    ENTITY_CATEGORY_DIAGNOSTIC,
)

from .. import CONF_BUS_T4_ID, BusT4Component, bus_t4_ns

DEPENDENCIES = ['bus_t4']

BusT4IoState = bus_t4_ns.class_('BusT4IoState', cg.Component)

# Diagnostics register 0xD1 payload: key -> (byte, mask, device_class).
# Layout from gashtaan/nice-bidiwifi-firmware's diagnostics page (web.cpp).
IO_BITS = {
    'stop': (0, 0x01, None),
    'step_by_step': (0, 0x02, None),
    'open': (0, 0x04, None),
    'close': (0, 0x08, None),
    'loop_1': (0, 0x10, None),
    'loop_2': (0, 0x20, None),
    'open_limit_m1': (2, 0x01, None),
    'close_limit_m1': (2, 0x02, None),
    'open_limit_m2': (2, 0x04, None),
    'close_limit_m2': (2, 0x08, None),
    # Released by the manual unlock key; HA's lock class reads "on" as unlocked
    'unlock_m1': (2, 0x10, DEVICE_CLASS_LOCK),
    'unlock_m2': (2, 0x20, DEVICE_CLASS_LOCK),
    'motor_m1': (4, 0x01, DEVICE_CLASS_RUNNING),
    'motor_m2': (4, 0x02, DEVICE_CLASS_RUNNING),
    'thermal': (9, 0x01, DEVICE_CLASS_HEAT),
    'standby_active': (9, 0x04, None),
    'on_battery': (9, 0x08, DEVICE_CLASS_BATTERY),
    'position_error': (10, 0x01, DEVICE_CLASS_PROBLEM),
    'bluebus_error': (10, 0x02, DEVICE_CLASS_PROBLEM),
    'stop_error': (10, 0x04, DEVICE_CLASS_PROBLEM),
    'function_error': (10, 0x08, DEVICE_CLASS_PROBLEM),
    'settings_error': (10, 0x10, DEVICE_CLASS_PROBLEM),
}


def _bit_schema(device_class):
    kwargs = {'device_class': device_class} if device_class else {}
    return binary_sensor.binary_sensor_schema(entity_category=ENTITY_CATEGORY_DIAGNOSTIC, **kwargs)


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(BusT4IoState),
            cv.GenerateID(CONF_BUS_T4_ID): cv.use_id(BusT4Component),
            cv.Optional(CONF_UPDATE_INTERVAL, default='5s'): cv.positive_time_period_milliseconds,
            **{cv.Optional(key): _bit_schema(cls) for key, (_byte, _mask, cls) in IO_BITS.items()},
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.has_at_least_one_key(*IO_BITS),
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    bus = await cg.get_variable(config[CONF_BUS_T4_ID])
    cg.add(var.set_parent(bus))
    for key, (byte, mask, _cls) in IO_BITS.items():
        if key in config:
            sens = await binary_sensor.new_binary_sensor(config[key])
            cg.add(var.add_bit(sens, byte, mask))
