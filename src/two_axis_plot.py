import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.ticker import MultipleLocator, FuncFormatter
from collections import deque
import time
import math

# === CONFIG ===
PORT = '/dev/ttyACM0'  # Change to your Arduino port
BAUD = 115200
WINDOW_SECONDS = 600
MAX_SAMPLES = 6000  # 100Hz × 60s

# === Data Buffers ===
time_vals = deque(maxlen=MAX_SAMPLES)
roll_vals = deque(maxlen=MAX_SAMPLES)
rate_roll_vals = deque(maxlen=MAX_SAMPLES)
pitch_vals = deque(maxlen=MAX_SAMPLES)
rate_pitch_vals = deque(maxlen=MAX_SAMPLES)

# === Setup Serial and Plot ===
ser = serial.Serial(PORT, BAUD)
time.sleep(2)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6))

# Plot 1: Roll and Roll Rate
line_roll, = ax1.plot([], [], label='Roll (rad)', color='blue')
line_rate_roll, = ax1.plot([], [], label='Roll Rate (rad/s)', color='green')
ax1.set_ylabel('Roll / Rate')
ax1.set_title('Roll and Roll Rate (Last 60 Seconds)')
ax1.grid(True, which='both', linestyle='--', linewidth=0.5, alpha=0.7)

# Plot 2: Pitch and Pitch Rate
line_pitch, = ax2.plot([], [], label='Pitch (rad)', color='orange')
line_rate_pitch, = ax2.plot([], [], label='Pitch Rate (rad/s)', color='red')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Pitch / Rate')
ax2.set_title('Pitch and Pitch Rate (Last 60 Seconds)')
ax2.grid(True, which='both', linestyle='--', linewidth=0.5, alpha=0.7)

# === Formatter for π ===
def pi_formatter(x, pos):
    denominator = 8
    multiple = round(x / math.pi * denominator)
    if multiple == 0:
        return "0"
    elif multiple == 1:
        return "π/{}".format(denominator)
    elif multiple == -1:
        return "-π/{}".format(denominator)
    else:
        return f"{multiple}π/{denominator}"

# === Apply tick format and grid lines every π/8 ===
pi_step = math.pi / 8
for ax in (ax1, ax2):
    ax.set_ylim(-4.5, 4.5)
    ax.yaxis.set_major_locator(MultipleLocator(pi_step))
    ax.yaxis.set_major_formatter(FuncFormatter(pi_formatter))
    for y in [i * pi_step for i in range(-12, 13)]:
        ax.axhline(y=y, color='gray', linestyle='--', linewidth=0.4)
    ax.legend()

start_time = None

# === Animation Update ===
def update(frame):
    global start_time

    while ser.in_waiting:
        try:
            line = ser.readline().decode().strip()
            t_raw, roll, rate_roll, pitch, rate_pitch = map(float, line.split(","))

            if start_time is None:
                start_time = t_raw

            t = (t_raw - start_time) / 1000.0  # ms → seconds

            time_vals.append(t)
            roll_vals.append(roll)
            rate_roll_vals.append(rate_roll)
            pitch_vals.append(pitch)
            rate_pitch_vals.append(rate_pitch)

        except:
            pass  # Skip malformed lines

    if len(time_vals) > 1:
        # Update data for both plots
        line_roll.set_data(time_vals, roll_vals)
        line_rate_roll.set_data(time_vals, rate_roll_vals)
        line_pitch.set_data(time_vals, pitch_vals)
        line_rate_pitch.set_data(time_vals, rate_pitch_vals)

        for ax in (ax1, ax2):
            ax.set_xlim(max(0, time_vals[-1] - WINDOW_SECONDS), time_vals[-1])

    return line_roll, line_rate_roll, line_pitch, line_rate_pitch

ani = animation.FuncAnimation(fig, update, interval=50)
plt.tight_layout()
plt.show()
