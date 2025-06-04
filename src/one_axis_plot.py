import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import time
import math

# === CONFIG ===
PORT = "/dev/ttyUSB1"  # Change to your Arduino port
BAUD = 115200
WINDOW_SECONDS = 60
MAX_SAMPLES = 6000  # Approx. 100Hz × 60s

# === Data Buffers ===
time_vals = deque(maxlen=MAX_SAMPLES)
roll_vals = deque(maxlen=MAX_SAMPLES)
rate_vals = deque(maxlen=MAX_SAMPLES)

# === Setup Serial and Plot ===
ser = serial.Serial(PORT, BAUD)
time.sleep(2)

fig, ax = plt.subplots()
(line_roll,) = ax.plot([], [], label="Roll (rad)", color="blue")
(line_rate,) = ax.plot([], [], label="Rate (rad/s)", color="green")
ax.legend()
ax.set_xlabel("Time (s)")
ax.set_ylabel("Value")
ax.set_title("Roll and Gyro Rate (Last 60 Seconds)")
ax.grid(True, which="both", linestyle="--", linewidth=0.5, alpha=0.7)

# Reference lines for -π, 0, π
for y in [-2 * math.pi, -math.pi, 0, math.pi, 2 * math.pi]:
    ax.axhline(y=y, color="gray", linestyle="--", linewidth=0.4)

start_time = None


def update(frame):
    global start_time

    while ser.in_waiting:
        try:
            line = ser.readline().decode().strip()
            t_raw, roll, rate = map(float, line.split(","))

            if start_time is None:
                start_time = t_raw

            t = (t_raw - start_time) / 1000.0  # Convert ms → seconds

            time_vals.append(t)
            roll_vals.append(roll)
            rate_vals.append(rate)

        except:
            pass  # skip malformed line

    if len(time_vals) > 1:
        line_roll.set_data(time_vals, roll_vals)
        line_rate.set_data(time_vals, rate_vals)

        ax.set_xlim(max(0, time_vals[-1] - WINDOW_SECONDS), time_vals[-1])
        ax.set_ylim(-2 * math.pi, 2 * math.pi)

    return line_roll, line_rate


ani = animation.FuncAnimation(fig, update, interval=50)
plt.tight_layout()
plt.show()
