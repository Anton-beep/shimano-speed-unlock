# Half-Speed Repeater — Arduino (ATmega32U4)

Reads the real magnet-on-wheel sensor (`scope/` wiring) and re-emits its pulse
train to the controller (`emulator/` wiring) at **half the rate** → controller
shows **half** the true speed.

The controller measures speed from the pulse RATE (period), so dropping the rate
by 2 halves the reading — regardless of wheel diameter, magnet geometry, or
acceleration. Done by relaying every 2nd input pulse (exact level/width copied)
and swallowing the one in between.

## Wiring

```
sensor wire A --> A0        (input, internal pullup; idle ~1023, magnet ~0)
sensor wire B --> GND
A2            --> controller signal line
Arduino GND   --> controller GND        (shared ground, required)
```

Sensor GND, Arduino GND, controller GND all common.

## Flash

```bash
FQBN=arduino:avr:leonardo
arduino-cli compile --fqbn $FQBN emulate-with-sensor && \
  arduino-cli upload --fqbn $FQBN -p /dev/ttyACM0 emulate-with-sensor
```

Upload fails? 32U4 needs bootloader: double-tap RST, re-run upload within ~8s.

## Tuning

- `DIVISOR` — pulses swallowed per emit (2 = half speed).
- `TH_LOW` / `TH_HIGH` — hysteresis thresholds; widen if sensor bounces.
- `PUSH_PULL` — 0 = open-collector (faithful), 1 = drive both rails.
