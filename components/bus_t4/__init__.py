import esphome.codegen as cg
from esphome.components import uart
# Aliased: importing this package's own sensor/ platform rebinds the name `sensor` here
from esphome.components import sensor as sensor_
import esphome.config_validation as cv
from esphome.const import (
    CONF_ADDRESS,
    CONF_ID,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_TOTAL_INCREASING,
)
DEPENDENCIES = ['uart']
# The hub's header pulls in sensor.h for the optional bus_errors counter
AUTO_LOAD = ['sensor']

bus_t4_ns = cg.esphome_ns.namespace('bus_t4')
BusT4Component = bus_t4_ns.class_('BusT4Component', cg.Component, uart.UARTDevice)

CONF_BUS_T4_ID = 'bus_t4_id'
CONF_BUS_ERRORS = 'bus_errors'

CONFIG_SCHEMA = cv.All(
    cv.Schema({
        cv.GenerateID(): cv.declare_id(BusT4Component),
        cv.Optional(CONF_ADDRESS, default=0x5090): cv.hex_uint16_t,
        cv.Optional(CONF_BUS_ERRORS): sensor_.sensor_schema(
            icon='mdi:alert-circle-outline',
            accuracy_decimals=0,
            state_class=STATE_CLASS_TOTAL_INCREASING,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
    })
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)

FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    "bus_t4_uart",
    require_tx=True,
    require_rx=True,
    baud_rate=19200,
    data_bits=8,
    parity="NONE",
    stop_bits=1,
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    cg.add(var.set_address(config[CONF_ADDRESS]))
    if CONF_BUS_ERRORS in config:
        cg.add(var.set_bus_errors_sensor(await sensor_.new_sensor(config[CONF_BUS_ERRORS])))
