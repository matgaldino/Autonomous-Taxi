# Autonomous Taxi

Autonomous mobile robot project based on ROS 2.

The system uses a Raspberry Pi for high-level robotics tasks and an STM32 microcontroller for low-level real-time control.

## Hardware

| Component | Model |
|---|---|
| Main computer | Raspberry Pi 4 Model B |
| LiDAR | SLAMTEC RPLIDAR A2M8 |
| Microcontroller | STM32 Nucleo L432KC |
| Controller | 8BitDo Ultimate 2C Wireless Controller |

## Software

- Ubuntu Server 26.04 ARM64
- ROS 2 Lyrical
- SLAMTEC `sllidar_ros2`
- colcon
- rosdep
- vcstool

## Repository Structure

```text
Autonomous-Taxi/
├── dependencies.repos
├── patches/
│   └── sllidar_ros2-lyrical.patch
├── scripts/
│   └── setup_dependencies.sh
├── ros2_ws/
│   └── src/
│       ├── taxi_examples/
│       ├── taxi_teleop/
│       └── third_party/       # generated, not tracked by Git
└── README.md
```

## External Dependencies

Third-party ROS packages are declared in `dependencies.repos`. They are downloaded with vcstool and are not committed directly to this repository.

The SLAMTEC ROS 2 driver currently requires a small compatibility patch for ROS 2 Lyrical, located at: `patches/sllidar_ros2-lyrical.patch`

The complete dependency setup is automated by:

```bash
./scripts/setup_dependencies.sh
```

Before running the script for the first time, rosdep must be initialized:

```bash
sudo rosdep init
rosdep update
```

## Build

Source ROS 2:

```bash
source /opt/ros/lyrical/setup.bash
```

Configure external dependencies:

```bash
./scripts/setup_dependencies.sh
```

Build the workspace:

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## RPLIDAR A2M8

The LiDAR is connected through USB and normally appears as `/dev/ttyUSB0`.
The A2M8 uses a serial baud rate of `115200`.

Start the ROS 2 driver:

```bash
ros2 launch sllidar_ros2 sllidar_a2m8_launch.py
```

The driver publishes to `/scan` using `sensor_msgs/msg/LaserScan`.

Basic validation commands:

```bash
ros2 topic info /scan
ros2 topic hz /scan
ros2 topic echo /scan --once
```

*The current hardware test produced valid 360-degree scans using the Sensitivity scan mode.*

## Gamepad Teleoperation

Manual teleoperation is available through a USB/wireless gamepad (8BitDo Ultimate 2C). 

The controller mapping is stored in:
`ros2_ws/src/taxi_teleop/config/8bitdo_ultimate_2c.yaml`

**Current mapping:**
- **Left stick horizontal:** steering
- **RT:** forward throttle
- **LT:** reverse throttle
- **LB:** deadman / enable button

Start teleoperation with:

```bash
cd ros2_ws
source install/setup.bash
ros2 launch taxi_teleop gamepad_teleop.launch.py
```

The resulting vehicle command is published on `/cmd_vel` using `geometry_msgs/msg/Twist`.

## Vehicle Control

Low-level vehicle control is handled by an STM32 Nucleo-L432KC. The Raspberry Pi runs the ROS 2 control stack and sends normalized commands to the STM32 through the ST-LINK Virtual COM Port.

### Control Architecture

```text
8BitDo Controller
      |
      v
   joy_node
      |
    /joy
      |
      v
 taxi_teleop
      |
  /cmd_vel
      |
      v
taxi_serial_bridge
      |
 USB Serial
 /dev/ttyACM0
      |
      v
 STM32 Nucleo
   |       |
 steering  ESC
```

### STM32 Connections

| Function | STM32 Pin | Peripheral |
| :--- | :--- | :--- |
| Steering servo signal | PA8 / D9 | TIM1_CH1 |
| ESC signal | PA11 / D10 | TIM1_CH4 |
| Raspberry Pi communication | ST-LINK USB | USART2 VCP |

*Both PWM channels operate at 50 Hz.*

### Calibration

**Steering Calibration:**
*   **1100 µs** — steering endpoint
*   **1400 µs** — mechanical center
*   **1700 µs** — steering endpoint

The steering direction is inverted in firmware so that ROS conventions are preserved (`angular.z > 0` $\rightarrow$ left, `angular.z < 0` $\rightarrow$ right).

**ESC Calibration:**
*   **1500 µs** — neutral
*   **~1600 µs** — forward starts
*   **1900 µs** — full forward
*   **~1200 µs** — reverse region
*   **1100 µs** — reverse endpoint

> **Note:** The ESC requires an explicit neutral command when changing driving direction. This behavior is handled automatically by the STM32 firmware.

### Serial Protocol

Commands sent by the Raspberry Pi have the following format:

```text
C,<steering>,<throttle>
```

Where both values are normalized integers ranging from `-1000` to `1000`. 

**Examples:**
*   `C,0,0`
*   `C,500,0`
*   `C,0,500`
*   `C,0,-500`

The STM32 converts these normalized values into calibrated PWM signals.

### Safety

The control chain contains multiple safety layers:

*   **Deadman Switch:** `taxi_teleop` commands zero velocity immediately if the LB deadman button is released.
*   **ROS 2 Watchdog:** A joystick watchdog commands zero velocity if `/joy` messages stop arriving.
*   **Bridge Watchdog:** `taxi_serial_bridge` outputs neutral commands if `/cmd_vel` stops arriving.
*   **Hardware Watchdog:** The STM32 has a 500 ms serial watchdog that returns steering to center and the ESC to neutral if communication with the Raspberry Pi is lost.
*   **Isolation:** The ESC/servo power system uses the ESC BEC (measured at 6 V). The STM32 remains USB-powered and shares only ground with the vehicle electronics.

## Current Status

**Validated Milestones:**

- [x] ROS 2 Lyrical running on Raspberry Pi and development workstation
- [x] Reproducible external dependency setup
- [x] RPLIDAR A2M8 communication and `/scan` publishing 
- [x] 8BitDo gamepad teleoperation (`/joy` to `/cmd_vel`)
- [x] Raspberry Pi to STM32 USB serial communication (`taxi_serial_bridge`)
- [x] Normalized serial vehicle-control protocol
- [x] STM32 communication watchdog
- [x] Steering servo and ESC PWM control
- [x] End-to-end gamepad steering and throttle control (physical vehicle integration)

*The vehicle control stack is currently configured with a conservative teleoperation speed limit for bench testing.*