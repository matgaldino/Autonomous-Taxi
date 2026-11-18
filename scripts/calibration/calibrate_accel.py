import serial
import statistics
import time

N = 500
G = 16384.0
MAX_STD = 300
STABILIZATION_TIME = 5

positions = ["+X", "-X", "+Y", "-Y", "+Z", "-Z"]
results = {}

with serial.Serial("/dev/ttyACM0", 115200, timeout=1) as ser:

    for position in positions:

        while True:
            input(f"\nPosition {position}. Press Enter...")

            # Flush old serial data
            ser.reset_input_buffer()

            print("Discarding initial samples...")

            deadline = time.monotonic() + STABILIZATION_TIME

            while time.monotonic() < deadline:
                ser.readline()

            ser.reset_input_buffer()

            # Collect samples
            samples = []
            start = time.monotonic()

            while len(samples) < N:

                line = ser.readline().decode(
                    errors="ignore"
                ).strip()

                if line.startswith("IMU A=") and " G=" in line:
                    try:
                        accel = line.split(" G=")[0][6:]
                        ax, ay, az = map(int, accel.split(","))
                    except ValueError:
                        continue

                    samples.append((ax, ay, az))

                    if len(samples) % 100 == 0:
                        print(f"{len(samples)}/{N}")

            elapsed = time.monotonic() - start

            # Compute statistics
            axes = list(zip(*samples))

            means = [statistics.mean(v) for v in axes]
            stds = [statistics.stdev(v) for v in axes]
            minimums = [min(v) for v in axes]
            maximums = [max(v) for v in axes]

            print("\nMean:", means)
            print("Std: ", stds)
            print("Min: ", minimums)
            print("Max: ", maximums)
            print(f"Duration: {elapsed:.2f} s")

            # Validate stability
            if max(stds) > MAX_STD:
                print("Unstable measurements. Repeat position.")
                continue

            # Validate orientation
            axis = "XYZ".index(position[1])
            sign = 1 if position[0] == "+" else -1

            if sign * means[axis] < 10000:
                print("Incorrect orientation. Repeat.")
                continue

            results[position] = means
            print("Position validated.")
            break

print("\n=== ACCELEROMETER CALIBRATION ===")

for i, axis in enumerate("XYZ"):

    positive = results["+" + axis][i]
    negative = results["-" + axis][i]

    bias = (positive + negative) / 2
    scale = (2 * G) / (positive - negative)

    print(f"ACCEL_BIAS_{axis}  = {bias:.4f}")
    print(f"ACCEL_SCALE_{axis} = {scale:.6f}")