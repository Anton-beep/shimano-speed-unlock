# Sensor Oscilloscope — Arduino Leonardo (ATmega32U4)

Stream analog reads of a 2-wire magnet sensor (reed / Hall switch) over USB
serial and view live on the laptop.

## Wiring (2-wire sensor)

```
sensor wire A --> A0
sensor wire B --> GND
```

No external resistor — code enables the chip's internal pullup (`INPUT_PULLUP`,
~20-50k). Logic is inverted: switch OPEN -> A0 ~1023 (HIGH), magnet CLOSES
switch -> A0 ~0 (LOW). Reading analog shows edges, bounce, and noise, not just
0/1.

Internal pullup is weaker than a 10k = slightly slower edges and more noise
pickup, fine at ~1 kHz. For long wires or clean fast edges, add an external 10k
pulldown instead (A->5V, B->A0 and ->GND via 10k) and use `INPUT`.

## 1. Install toolchain (Arch)

```bash
sudo pacman -S arduino-cli
arduino-cli config init
arduino-cli core update-index
arduino-cli core install arduino:avr
```

Serial port access (relog after):

```bash
sudo usermod -aG uucp $USER
```

## 2. Pick board + port

```bash
arduino-cli board list
```

Should list as `Arduino Leonardo`. Set FQBN:

```bash
FQBN=arduino:avr:leonardo
```

## 3. Flash

```bash
arduino-cli compile --fqbn $FQBN scope
arduino-cli upload  --fqbn $FQBN -p /dev/ttyACM0 scope   # replace port
```

Upload fails? 32U4 needs bootloader: double-tap RST (short RST to GND twice
fast), then re-run upload within ~8s. Port may change during bootloader.

## 4. View on laptop

### Option A — raw, no code
```bash
arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200
```
Or use Arduino IDE Serial Plotter.

### Option B — live plot + CSV log
```bash
python -m venv venv && source venv/bin/activate
pip install pyserial matplotlib
python plot.py /dev/ttyACM0
```
Live curve on screen, every sample saved to `capture.csv`.

## 5. Test sensor

1. Flash, run plotter.
2. Move magnet near/far.
3. Read trace:
   - sharp 0<->1023 step = clean switch
   - spikes on edge = contact bounce
   - slow ramp = analog Hall output
   - floating noise = pullup not enabled / wrong wiring
