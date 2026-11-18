import serial
import statistics
import math
import time

N = 100

with serial.Serial("/dev/ttyACM0", 115200, timeout=1) as ser:

    for position in ["Horizontal", "Tilt forward", "Tilt sideways"]:

        input(f"\n{position}. Press Enter...")
        time.sleep(3)
        ser.reset_input_buffer()

        accel = []
        gyro = []

        while len(accel) < N:
            line = ser.readline().decode(errors="ignore").strip()

            if not line.startswith("IMU A="):
                continue

            try:
                a, g = line[6:].split(" G=")
                accel.append(tuple(map(int, a.split(","))))
                gyro.append(tuple(map(int, g.split(","))))
            except ValueError:
                continue

        norms = [
            math.sqrt(x*x + y*y + z*z) / 16384
            for x, y, z in accel
        ]

        print("Acceleration norm (g):",
              round(statistics.mean(norms), 4))

        print("Acceleration std (g):",
              round(statistics.stdev(norms), 4))

        print("Gyroscope mean (LSB):",
              [round(statistics.mean(v), 2)
               for v in zip(*gyro)])