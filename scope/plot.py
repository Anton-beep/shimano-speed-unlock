#!/usr/bin/env python3
"""Live plot + CSV log of Arduino sensor scope.

Usage: python plot.py [PORT]
  PORT defaults to /dev/ttyACM0
Saves every sample to capture.csv.
"""
import sys
import collections
import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
BAUD = 115200
N = 2000  # points shown on screen

ser = serial.Serial(PORT, BAUD, timeout=1)
ys = collections.deque([0] * N, maxlen=N)
log = open("capture.csv", "w")
log.write("micros,value\n")

fig, ax = plt.subplots()
(line,) = ax.plot(range(N), list(ys))
ax.set_ylim(-50, 1100)
ax.set_ylabel("A0 (0-1023)")
ax.set_xlabel("samples")
ax.set_title("sensor scope")


def update(_):
    for _ in range(200):  # drain serial buffer each frame
        raw = ser.readline().decode(errors="ignore").strip()
        if not raw or "," not in raw:
            continue
        _, _, v = raw.partition(",")
        try:
            ys.append(int(v))
            log.write(raw + "\n")
        except ValueError:
            pass
    line.set_ydata(list(ys))
    return (line,)


ani = FuncAnimation(fig, update, interval=30, blit=True)
plt.show()
log.close()
ser.close()
