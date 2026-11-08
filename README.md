# esphome-redlink

I built this to control my Honeywell RedLINK thermostat locally through ESPHome,
without the Internet Gateway. It uses an ESP32-S3 and a 902–928 MHz CC1101 radio.

Tested with a **TH8320R1003 thermostat**, **THM5421R1021 EIM**, and
**ESPHome 2026.10.2**.

**This is experimental.** Hop order is learned passively for your configured
`network_id`; no installation's hop table is built in. The physical candidate
frequencies/sync rule are measured protocol assumptions, not universal RF calibration.

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

1. Build and upload with `receive_only: true`. Keep a previous firmware image for rollback.
2. Wait for **Discovery Status** to show `ready; saved` and **Clock Locked** to turn on.
   Cold discovery can take hours (roughly two hours even in idealized scans), not seconds.
3. Set `receive_only: false`, rebuild, and upload. Wait for fresh cache validation,
   then deliberately turn on **RedLINK Transmit Enabled**. Refresh Status checks an existing association.
4. Only if enrollment is needed, open **Add Device** on the thermostat and press
   **RedLINK Pair Gateway**. Return home, refresh, and check readback (up to three minutes).

Pairing replaces the existing gateway. Don't reset or unpair the thermostat or
EIM. A failed/unverified attempt can still replace the gateway; there's no
automatic rollback. Keep the original gateway around so you can pair it again.
The example turns TX off after every reboot.

Discovery requires two independent CRC-valid clock counters for every slot and
rejects conflicting or duplicate channel assignments. Repeated packets do not count.
`Discovery:` logs show seen/confirmed progress; `EIM clock acquired:` logs show
logical slot, physical channel, and counter. A clock lock alone does **not** authorize TX.
Commands are rejected while learning, rather than queued into short transaction timeouts.

Only a complete map is saved, with version, network, tuning reference, and checksum.
After reboot two independent matching live clocks must validate the cache before TX.
Software restarts try a retained receive-channel hint for up to 20 seconds.
Power loss or a stale hint falls back to scanning; neither bypasses clock validation.
Clock acquisition/reacquisition also checks counter progression against elapsed RF
time; `Clock provisional:` is not a lock. After an EIM reset or long outage,
`Clock epoch reacquired` retains the confirmed map, drops single-observation credit,
and requires a fresh timed pair rather than waiting for the counter to wrap.
**Rediscover Channels** is passive, disables TX, cancels pending commands, and leaves
the previous saved map untouched until a complete replacement is ready. Toggle
Transmit Enabled off/on afterward to opt back in. Conflicts block TX until
rediscovery; corrupt/incompatible cache starts cold discovery. This is separate from pairing.
Boot never sends HVAC writes. `frequency` is the channel-10 calibration reference;
logical slot zero can use any learned physical channel. `sync_word` overrides are rejected.

For a safe live check, watch a receive-only node for new seen/confirmed entries and
verify there are no `TX type=` logs. Let it finish before enabling TX; reboot to check
`cache awaiting fresh clocks` changes to `ready; saved`. RF loss can extend discovery
beyond hours; a short rehearsal proves reception, not completion of all 50 slots.

## Controls

- Heat/cool targets, **55–80°F**, with at least 2°F between them.
- Heat, Cool, Auto, Off, and Emergency Heat through the **System Mode** dropdown.
- Fan Auto, On, and Circulate.
- Schedule, temporary/permanent holds, and separate heat/cool hold deadlines.
- Current room temperature and humidity.

Targets update optimistically in Home Assistant, then readback confirms them.
After boot, climate state stays unknown until the first actual readback.
Commands aren't instant; the thermostat still has its own radio cycle.
I acknowledge status replies promptly so the EIM can stop repeating them.
I haven't decoded actual heating/cooling activity.
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
