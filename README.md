# Autonomous Taxi

Autonomous mobile robot project based on ROS 2.

The system uses a Raspberry Pi for high-level robotics tasks and an STM32 microcontroller for low-level real-time control.

## Hardware

| Component | Model |
|---|---|
| Main computer | Raspberry Pi 4 Model B |
| LiDAR | SLAMTEC RPLIDAR A2M8 |
| Microcontroller | STM32 Nucleo L432KC |

## Software

- Ubuntu Server 26.04 ARM64
- ROS 2 Lyrical
- SLAMTEC `sllidar_ros2`
- colcon
- rosdep
- vcstool

## Repository structure

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
│       └── third_party/       # generated, not tracked by Git
└── README.md
```

## External dependencies

Third-party ROS packages are declared in `dependencies.repos`. They are downloaded with vcstool and are not committed directly to this repository.

The SLAMTEC ROS 2 driver currently requires a small compatibility patch for ROS 2 Lyrical:
`patches/sllidar_ros2-lyrical.patch`

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

The LiDAR is connected through USB and normally appears as: `/dev/ttyUSB0`
The A2M8 uses a serial baud rate of: `115200`

Start the ROS 2 driver:
```bash
ros2 launch sllidar_ros2 sllidar_a2m8_launch.py
```

The driver publishes `/scan` using `sensor_msgs/msg/LaserScan`.

Basic validation commands:
```bash
ros2 topic info /scan
ros2 topic hz /scan
ros2 topic echo /scan --once
```

The current hardware test produced valid 360-degree scans using the Sensitivity scan mode.

## Current status

RPLIDAR A2M8 hardware communication and ROS 2 integration have been validated. Next steps include LiDAR visualization, robot coordinate frames, mapping and navigation.