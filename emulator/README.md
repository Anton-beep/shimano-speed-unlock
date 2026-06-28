```bash
FQBN=arduino:avr:leonardo
arduino-cli compile --fqbn $FQBN emulator && arduino-cli upload  --fqbn $FQBN -p /dev/ttyACM0 emulator
```
