import serial
import time
import csv
import statistics

DURATION = 180
samples = []

with serial.Serial("/dev/ttyACM0", 115200, timeout=1) as ser:

    input("Press Enter to start calibration...")

    ser.reset_input_buffer()
    start = time.monotonic()

    while time.monotonic() - start < DURATION:

        line = ser.readline().decode(
            errors="ignore"
        ).strip()

        if not line.startswith("MAG M="):
            continue

        try:
            mx, my, mz = map(int, line[6:].split(","))
        except ValueError:
            continue

        samples.append((mx, my, mz))

        if len(samples) % 100 == 0:
            print(f"{len(samples)} samples")

if len(samples) < 100:
    raise RuntimeError("Insufficient samples")

with open("/tmp/mag_calibration.csv", "w") as f:
    writer = csv.writer(f)
    writer.writerow(["mx", "my", "mz"])
    writer.writerows(samples)

axes = list(zip(*samples))

minimums = [min(v) for v in axes]
maximums = [max(v) for v in axes]

bias = [
    (hi + lo) / 2
    for lo, hi in zip(minimums, maximums)
]

radii = [
    (hi - lo) / 2
    for lo, hi in zip(minimums, maximums)
]

if min(radii) <= 0:
    raise RuntimeError("Insufficient rotation")

average_radius = statistics.mean(radii)

scale = [average_radius / r for r in radii]

print("\nSamples:", len(samples))
print("Min:", minimums)
print("Max:", maximums)
print("Radii:", radii)

print("\n=== MAGNETOMETER CALIBRATION ===")

for i, axis in enumerate("XYZ"):
    print(f"MAG_BIAS_{axis}  = {bias[i]:.4f}")
    print(f"MAG_SCALE_{axis} = {scale[i]:.6f}")