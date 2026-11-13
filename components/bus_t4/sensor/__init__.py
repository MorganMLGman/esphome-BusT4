import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import CONF_LENGTH, CONF_OFFSET, CONF_UPDATE_INTERVAL

from .. import CONF_BUS_T4_ID, BusT4Component, bus_t4_ns

DEPENDENCIES = ['bus_t4']

BusT4RegisterSensor = bus_t4_ns.class_('BusT4RegisterSensor', sensor.Sensor, cg.Component)

CONF_REGISTER = 'register'

CONFIG_SCHEMA = (
    sensor.sensor_schema(BusT4RegisterSensor, icon='mdi:register', accuracy_decimals=0)
    .extend(
        {
            cv.GenerateID(CONF_BUS_T4_ID): cv.use_id(BusT4Component),
            cv.Required(CONF_REGISTER): cv.hex_uint8_t,
            # Skips a prefix field in the reply before decoding
            cv.Optional(CONF_OFFSET, default=0): cv.int_range(min=0, max=15),
            # Bytes combined big-endian; 0 takes the rest of the reply, capped at 4
            cv.Optional(CONF_LENGTH, default=0): cv.int_range(min=0, max=4),
            cv.Optional(CONF_UPDATE_INTERVAL, default='60s'): cv.positive_time_period_milliseconds,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = await sensor.new_sensor(config)
    await cg.register_component(var, config)

    bus = await cg.get_variable(config[CONF_BUS_T4_ID])
    cg.add(var.set_parent(bus))
    cg.add(var.set_register(config[CONF_REGISTER]))
    cg.add(var.set_window(config[CONF_OFFSET], config[CONF_LENGTH]))
