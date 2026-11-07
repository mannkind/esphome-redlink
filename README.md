# esphome-redlink

I built this to control my Honeywell RedLINK thermostat locally through ESPHome,
without the Internet Gateway. It uses an ESP32-S3 and a 902–928 MHz CC1101 radio.

Tested with a **TH8320R1003 thermostat**, **THM5421R1021 EIM**, and
**ESPHome 2026.10.2**.

**This is experimental. The channel map is specific to my network (`0x389e`).**
Changing the IDs alone won't make it work on another installation. I haven't
worked out automatic discovery or channel maps for other networks yet.

## Setup

Start with [example.yaml](example.yaml). Change the credentials, wiring, and IDs,
and replace its local component source with:

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/mannkind/esphome-redlink
      ref: main
    components: [redlink]
```

Use a 3.3 V CC1101 module and antenna for the correct band. The example pins are
board-specific; GPIO45/46 have boot-strapping constraints.

1. Build and upload. Leave TX disabled until **Radio Healthy** and **Clock Locked** are on.
2. Set `receive_only: false`, rebuild, and upload. Turn on **RedLINK Transmit Enabled**.
3. Open **Add Device** on the thermostat, then press **RedLINK Pair Gateway** in Home Assistant.
4. Return to the thermostat's home screen, press **RedLINK Refresh Status**, and check the settings. Readback can take up to three minutes.

Pairing replaces the existing gateway. Don't reset or unpair the thermostat or
EIM. A failed/unverified attempt can still replace the gateway; there's no
automatic rollback. Keep the original gateway around so you can pair it again.
The example turns TX off after every reboot.

## Controls

- Heat/cool targets, **55–80°F**, with at least 2°F between them.
- Heat, Cool, Auto, Off, and Emergency Heat through the **System Mode** dropdown.
- Fan Auto, On, and Circulate.
- Schedule, temporary/permanent holds, and separate heat/cool hold deadlines.
- Current room temperature and humidity.

Targets update optimistically in Home Assistant, then readback confirms them.
Radio commands aren't instant. I haven't decoded actual heating/cooling activity.
The thermostat and EIM still control the equipment; this isn't a safety controller.

## Development

Tools: mise, Node.js, uv, Lefthook, and Cppcheck. `mise install` supplies Prettier
and ShellCheck. No npm project or task runner.

```sh
mise install
git config --local core.hooksPath .git/hooks
lefthook install --force
bash tools/format.sh
bash tools/check.sh
bash tools/test.sh
uvx --from esphome==2026.10.2 esphome compile example.yaml
```

Hooks check formatting and static analysis on commits and pushes; pushes also
run the host tests. CI checks both and builds the firmware.
I prefer early returns over wrapping a function's work in another `if`.

## Credits

[Len Shustek's RedLINK-Sniffer](https://github.com/LenShustek/RedLINK-Sniffer)
was my starting reference. This component is [MIT licensed](LICENSE).
