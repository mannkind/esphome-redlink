import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import binary_sensor, climate, select, spi, text_sensor
from esphome.const import CONF_ID

from . import redlink_ns

DEPENDENCIES = ["spi", "esp32"]
AUTO_LOAD = ["binary_sensor", "select", "text_sensor"]
RedLinkClimate = redlink_ns.class_(
    "RedLinkClimate", cg.Component, climate.Climate, spi.SPIDevice
)
RedLinkSystemModeSelect = redlink_ns.class_("RedLinkSystemModeSelect", select.Select)


def noninverted_input(value):
    value = pins.internal_gpio_input_pin_schema(value)
    if value["inverted"]:
        raise cv.Invalid("RedLINK radio pins must not be inverted")
    return value


CONFIG_SCHEMA = (
    climate.climate_schema(RedLinkClimate)
    .extend(
        {
            cv.Required("gdo0_pin"): noninverted_input,
            cv.Required("miso_pin"): noninverted_input,
            cv.Required("network_id"): cv.All(
                cv.hex_uint16_t, cv.int_range(min=1, max=0xFFFE)
            ),
            cv.Required("gateway_id"): cv.hex_uint16_t,
            # Reference carrier for physical channel 10, not logical slot zero.
            cv.Optional("frequency", default=904195310): cv.int_range(
                min=903199706, max=905805439
            ),
            cv.Optional("sync_word"): cv.invalid(
                "sync_word is derived from the physical candidate channel; remove this option"
            ),
            cv.Optional("tx_advance_us", default=800): cv.int_range(
                min=-10000, max=10000
            ),
            cv.Optional("receive_only", default=True): cv.boolean,
            cv.Optional("result"): text_sensor.text_sensor_schema(
                entity_category="diagnostic"
            ),
            cv.Optional("pairing_status"): text_sensor.text_sensor_schema(
                entity_category="diagnostic"
            ),
            cv.Optional("discovery_status"): text_sensor.text_sensor_schema(
                entity_category="diagnostic"
            ),
            cv.Optional("heat_hold_until"): text_sensor.text_sensor_schema(),
            cv.Optional("cool_hold_until"): text_sensor.text_sensor_schema(),
            cv.Optional("paired"): binary_sensor.binary_sensor_schema(
                entity_category="diagnostic"
            ),
            cv.Optional("acknowledged"): binary_sensor.binary_sensor_schema(
                entity_category="diagnostic"
            ),
            cv.Optional("verified"): binary_sensor.binary_sensor_schema(
                entity_category="diagnostic"
            ),
            cv.Optional("clock_locked"): binary_sensor.binary_sensor_schema(
                entity_category="diagnostic"
            ),
            cv.Optional("radio_healthy"): binary_sensor.binary_sensor_schema(
                entity_category="diagnostic"
            ),
            cv.Optional("command_pending"): binary_sensor.binary_sensor_schema(
                entity_category="diagnostic"
            ),
            cv.Optional("emergency_heat"): binary_sensor.binary_sensor_schema(),
            cv.Optional("system_mode"): select.select_schema(RedLinkSystemModeSelect),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(spi.spi_device_schema(cs_pin_required=True))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await climate.register_climate(var, config)
    await spi.register_spi_device(var, config)
    cg.add(var.set_gdo0_pin(await cg.gpio_pin_expression(config["gdo0_pin"])))
    for key in ("network_id", "gateway_id", "frequency", "tx_advance_us"):
        cg.add(getattr(var, f"set_{key}")(config[key]))
    cg.add(var.set_miso_pin(await cg.gpio_pin_expression(config["miso_pin"])))
    cg.add(var.set_receive_only(config["receive_only"]))
    for key in (
        "result",
        "pairing_status",
        "discovery_status",
        "heat_hold_until",
        "cool_hold_until",
    ):
        if key in config:
            cg.add(
                getattr(var, f"set_{key}_sensor")(
                    await text_sensor.new_text_sensor(config[key])
                )
            )
    for key in (
        "acknowledged",
        "verified",
        "clock_locked",
        "paired",
        "radio_healthy",
        "command_pending",
        "emergency_heat",
    ):
        if key in config:
            cg.add(
                getattr(var, f"set_{key}_sensor")(
                    await binary_sensor.new_binary_sensor(config[key])
                )
            )
    if "system_mode" in config:
        cg.add(
            var.set_system_mode_select(
                await select.new_select(
                    config["system_mode"],
                    var,
                    options=["Auto", "Heat", "Cool", "Off", "Emergency Heat"],
                )
            )
        )
