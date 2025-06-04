import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import time
import math

# === CONFIG ===
PORT = "/dev/ttyACM0"  # Change to your Arduino port
BAUD = 115200
WINDOW_SECONDS = 60
MAX_SAMPLES = 6000  # Approx. 100Hz × 60s

# === Data Buffers ===
time_vals = deque(maxlen=MAX_SAMPLES)
roll_vals = deque(maxlen=MAX_SAMPLES)
pitch_vals = deque(maxlen=MAX_SAMPLES)
roll_rate_vals = deque(maxlen=MAX_SAMPLES)
pitch_rate_vals = deque(maxlen=MAX_SAMPLES)

# === Setup Serial ===
ser = serial.Serial(PORT, BAUD)
time.sleep(2)

# === Setup Plot ===
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

# Roll and Pitch angles
(line_roll,) = ax1.plot([], [], label="Roll (rad)", color="blue")
(line_pitch,) = ax1.plot([], [], label="Pitch (rad)", color="red")
ax1.set_ylabel("Angle (rad)")
ax1.set_title("Roll and Pitch (Last 60s)")
ax1.legend()
ax1.grid(True, which="both", linestyle="--", linewidth=0.5, alpha=0.7)

# Reference lines
for y in [-2 * math.pi, -math.pi, 0, math.pi, 2 * math.pi]:
    ax1.axhline(y=y, color="gray", linestyle="--", linewidth=0.4)

# Gyro Rates
(line_roll_rate,) = ax2.plot([], [], label="Gyro Roll Rate (rad/s)", color="green")
(line_pitch_rate,) = ax2.plot([], [], label="Gyro Pitch Rate (rad/s)", color="orange")
ax2.set_xlabel("Time (s)")
ax2.set_ylabel("Rate (rad/s)")
ax2.set_title("Gyro Rates")
ax2.legend()
ax2.grid(True, which="both", linestyle="--", linewidth=0.5, alpha=0.7)

start_time = None


def update(frame):
    global start_time

    while ser.in_waiting:
        try:
            line = ser.readline().decode().strip()
            # Expected format: millis,roll,pitch,gyroRollRate,gyroPitchRate
            t_raw, roll, pitch, gyro_roll, gyro_pitch = map(float, line.split(","))

            if start_time is None:
                start_time = t_raw

            t = (t_raw - start_time) / 1000.0  # Convert ms → seconds

            time_vals.append(t)
            roll_vals.append(roll)
            pitch_vals.append(pitch)
            roll_rate_vals.append(gyro_roll)
            pitch_rate_vals.append(gyro_pitch)

        except ValueError:
            continue  # Ignore malformed line

    if len(time_vals) > 1:
        # Update angle plot
        line_roll.set_data(time_vals, roll_vals)
        line_pitch.set_data(time_vals, pitch_vals)
        ax1.set_xlim(max(0, time_vals[-1] - WINDOW_SECONDS), time_vals[-1])
        ax1.set_ylim(-5, 5)

        # Update rate plot
        line_roll_rate.set_data(time_vals, roll_rate_vals)
        line_pitch_rate.set_data(time_vals, pitch_rate_vals)
        ax2.set_xlim(ax1.get_xlim())
        ax2.set_ylim(-10, 10)

    return line_roll, line_pitch, line_roll_rate, line_pitch_rate


ani = animation.FuncAnimation(fig, update, interval=50)
plt.tight_layout()
plt.show()
