# D1 Jetson Orin USB-CANFD Deployment README

[中文版本](README_CN.md)


This README documents the D1 real-robot deployment flow for Jetson Orin Nano Super + Ubuntu 22.04 + ROS 2 Humble. The CAN bus uses a DaMiao `DM-USB2CANFD_Dual` USB-CAN FD adapter, and the IMU is `DM-IMU-L1`.

The document has been corrected for the current repository state: the robot name is now `d1`, ROS2 package metadata is already in place, and the real-robot CAN interfaces are already configured as `can1` and `can2`. No manual source patching is required for robot renaming, Gazebo removal, CAN interface remapping, or `package.xml` generation.

### Verified Status

Verified:

- ROS 2 Humble workspace builds successfully.
- `d1_description` has a valid ROS2 `package.xml`.
- `rl_sar` provides the real-robot executable `rl_real_d1`.
- `dm_imu_node` publishes `/imu/data`.
- After flashing SocketCAN firmware, `DM-USB2CANFD_Dual` enumerates as `can1` and `can2` on Jetson.
- The custom `5.15/gs_usb.ko` driver works for CAN FD on the Jetson 5.15 kernel.
- `d1-gs-usb.service`, `d1-canfd@.service`, and the udev rule can auto-load and configure CAN FD interfaces.
- `rl_real_d1` starts, FSM transitions work, and the ONNX policy can be loaded.

Still requires safe real-robot validation:

- Full CAN communication with real motors connected.
- Suspended-robot stand-up, sit-down, and locomotion tests.

### Requirements

| Item | Requirement |
|---|---|
| Board | Jetson Orin Nano Super |
| OS | Ubuntu 22.04 |
| ROS | ROS 2 Humble |
| Kernel | Jetson 5.15.x series |
| Robot | D1 |
| IMU | DM-IMU-L1 |
| CAN | DM-USB2CANFD_Dual |
| Inference | ONNX Runtime, CPU first |

If your kernel is not 5.15.x, rebuild and re-check the custom driver. If your CAN adapter is not `DM-USB2CANFD_Dual`, do not copy the CAN setup blindly.

### Repository Path

This README assumes the repository is located at:

```bash
~/project/d1_rl_sar
```

Clone:

```bash
mkdir -p ~/project
cd ~/project
git clone --recursive https://gitee.com/lookc4/d1_rl_sar.git
cd ~/project/d1_rl_sar/rl_sar
```

### Install Dependencies

Install ROS 2 Humble base tools:

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y ros-humble-ros-base ros-dev-tools
source /opt/ros/humble/setup.bash
```

Install project dependencies:

```bash
sudo apt install -y \
  git curl unzip locales software-properties-common \
  cmake g++ build-essential \
  libyaml-cpp-dev libeigen3-dev libboost-all-dev libspdlog-dev libfmt-dev libtbb-dev liblcm-dev \
  python3-colcon-common-extensions \
  ros-humble-rclcpp ros-humble-rclpy \
  ros-humble-geometry-msgs ros-humble-sensor-msgs ros-humble-std-msgs ros-humble-std-srvs \
  ros-humble-tf2 ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
  ros-humble-realtime-tools ros-humble-hardware-interface ros-humble-controller-interface \
  ros-humble-pluginlib ros-humble-urdf ros-humble-rcl-interfaces ros-humble-rosidl-default-generators \
  ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-joint-state-broadcaster \
  ros-humble-control-toolbox ros-humble-robot-state-publisher ros-humble-joint-state-publisher-gui \
  ros-humble-xacro ros-humble-teleop-twist-keyboard \
  can-utils xterm
```

### Download ONNX Runtime

Large runtime libraries under `rl_sar/library/` are not committed. If ONNX Runtime is missing, run:

```bash
cd ~/project/d1_rl_sar/rl_sar
bash scripts/download_inference_runtime.sh onnx
```

### Build ROS2 Workspace

```bash
cd ~/project/d1_rl_sar/rl_sar
source /opt/ros/humble/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=1
./build.sh
```

Check the build:

```bash
source install/setup.bash
colcon list --base-paths src
ros2 pkg executables rl_sar
ros2 pkg executables dm_imu
ldd install/lib/rl_sar/rl_real_d1 | grep 'not found' || true
```

Expected entries:

```text
d1_description
rl_sar rl_real_d1
dm_imu dm_imu_node
```

No output from the `ldd ... | grep 'not found'` command means required shared libraries are found.

### Source Setup Files Automatically

After a successful build, append this to `~/.bashrc`:

```bash
cat >> ~/.bashrc <<'EOF'

source /opt/ros/humble/setup.bash
if [ -f "$HOME/project/d1_rl_sar/rl_sar/install/setup.bash" ]; then
  source "$HOME/project/d1_rl_sar/rl_sar/install/setup.bash"
fi
EOF
```

Apply it to the current terminal:

```bash
source ~/.bashrc
```

### Verify IMU

Add serial permission if needed:

```bash
groups
sudo usermod -aG dialout $USER
newgrp dialout
```

Start IMU. The tested port is `/dev/ttyACM0`:

```bash
source /opt/ros/humble/setup.bash
source ~/project/d1_rl_sar/rl_sar/install/setup.bash
ros2 run dm_imu dm_imu_node --ros-args -p port:=/dev/ttyACM0 -p baud:=921600
```

In another terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/project/d1_rl_sar/rl_sar/install/setup.bash
ros2 topic echo /imu/data --once
```

A `sensor_msgs/msg/Imu` message means the IMU path is working.

### Flash USB-CANFD Firmware

`DM-USB2CANFD_Dual` usually does not ship in Linux SocketCAN mode. Before using SocketCAN, flash the socketcan firmware with DaMiao's official upgrade tool.

Reference firmware name:

```text
dm_usb2canfd_dual_gsusb_1004.enc
```

Before flashing, the device may appear as `ttyACM*` and `lsusb` may show `DaMiao-Tech DM-USB2FDCAN`.

After flashing and re-plugging, expected behavior:

- `lsusb` shows `1d50:606f OpenMoko ... CAN adapter`.
- Linux creates CAN network interfaces. If Jetson's onboard CAN uses `can0`, the USB dual-channel adapter usually becomes `can1` and `can2`.

### Build and Load `gs_usb`

For Jetson 5.15 kernels, use the custom driver under `5.15/`:

```bash
sudo apt update
sudo apt install -y build-essential nvidia-l4t-kernel-headers
uname -r
ls -ld /lib/modules/$(uname -r)/build

cd ~/project/d1_rl_sar/5.15
make clean
make

sudo modprobe can
sudo modprobe can_raw
sudo modprobe can_dev
sudo modprobe -r gs_usb 2>/dev/null || true
sudo insmod ./gs_usb.ko
```

If a repeated `insmod` prints `File exists`, the module is already loaded.

### Recommended CANFD Autoload

Install the autoload setup:

```bash
cd ~/project/d1_rl_sar/5.15/autoload
sudo bash install.sh
```

Installed files:

| File | Purpose |
|---|---|
| `/etc/modules-load.d/d1-can.conf` | Load `can`, `can_raw`, and `can_dev` at boot |
| `/etc/systemd/system/d1-gs-usb.service` | Load `gs_usb.ko` by absolute path |
| `/etc/udev/rules.d/85-d1-canfd.rules` | Watch CAN interfaces created by `gs_usb` |
| `/etc/systemd/system/d1-canfd@.service` | Configure `1M arbitration / 5M data / FD on` and bring the interface up |

Check:

```bash
systemctl --no-pager status d1-gs-usb.service
systemctl --no-pager status 'd1-canfd@can1.service' 'd1-canfd@can2.service'
ip -d link show can1
ip -d link show can2
```

Uninstall:

```bash
sudo bash ~/project/d1_rl_sar/5.15/autoload/uninstall.sh
```

After a Jetson kernel upgrade, rebuild the module:

```bash
cd ~/project/d1_rl_sar/5.15
make clean && make
sudo systemctl restart d1-gs-usb.service
```

### Manual CANFD Setup

If autoload is not installed:

```bash
cd ~/project/d1_rl_sar/5.15
sudo modprobe can
sudo modprobe can_raw
sudo modprobe can_dev
lsmod | grep -q '^gs_usb' || sudo insmod ./gs_usb.ko

sudo ip link set can1 down 2>/dev/null || true
sudo ip link set can2 down 2>/dev/null || true
sudo ip link set can1 type can bitrate 1000000 dbitrate 5000000 sample-point 0.75 dsample-point 0.875 fd on
sudo ip link set can2 type can bitrate 1000000 dbitrate 5000000 sample-point 0.75 dsample-point 0.875 fd on
sudo ip link set can1 up
sudo ip link set can2 up
```

### CANFD Loopback Test

Terminal 1:

```bash
sudo candump can2 -t a
```

Terminal 2:

```bash
sudo cansend can1 123##91122334455667788
```

If `candump` receives the frame, firmware, driver, and CAN FD wiring are basically working.

### CAN Interface Convention

The current `rl_real_d1.cpp` already uses:

| Logical bus | SocketCAN interface | Purpose |
|---|---|---|
| CAN1 | `can1` | First 6 motors |
| CAN2 | `can2` | Second 6 motors |

Do not apply old `can0/can1` to `can1/can2` sed patches again.

### Motor Zero Calibration

D1 uses DaMiao `DM6248P` motors. Motor zero offsets are stored in each motor's FLASH and survive power cycles. Calibrate on first deployment, after replacing motors, or after mechanical reassembly.

Preconditions:

- `can1` and `can2` are `UP`.
- Each bus uses motor IDs `0x01..0x06`.
- Robot is suspended and legs can move freely.
- All 12 joints are placed at URDF zero pose.
- No other CAN-using process is running.

Calibrate all motors:

```bash
source /opt/ros/humble/setup.bash
source ~/project/d1_rl_sar/rl_sar/install/setup.bash
ros2 run dmbot_serial set_zero_all
```

Single motor calibration:

```bash
ros2 run dmbot_serial set_zero_one can1 0x02
ros2 run dmbot_serial set_zero_one can2 5
```

Motor mapping:

| can1 ID | Joint | can2 ID | Joint |
|---|---|---|---|
| 0x01 | FL_hip | 0x01 | RL_hip |
| 0x02 | FL_thigh | 0x02 | RL_thigh |
| 0x03 | FL_calf | 0x03 | RL_calf |
| 0x04 | FR_hip | 0x04 | RR_hip |
| 0x05 | FR_thigh | 0x05 | RR_thigh |
| 0x06 | FR_calf | 0x06 | RR_calf |

After writing FLASH, physically power-cycle the motors and run `set_zero_all` again. The BEFORE table should show all joints close to zero.

### Run Real Robot

Safety requirement: for the first run, suspend the robot and verify the emergency stop path before entering locomotion.

One-command launch, for Jetson sessions with GUI/X11:

```bash
source /opt/ros/humble/setup.bash
source ~/project/d1_rl_sar/rl_sar/install/setup.bash
ros2 launch rl_sar rl_real_d1.launch.py
```

The launch file starts:

- `dm_imu_node`
- `rl_real_d1`

Default IMU parameters:

```text
port=/dev/ttyACM0
baud=921600
```

Override IMU port:

```bash
ros2 launch rl_sar rl_real_d1.launch.py port:=/dev/ttyACM1 baud:=921600
```

The launch file wraps `rl_real_d1` with `xterm -hold -e`, because keyboard control needs a real TTY. For pure SSH without X11, use two terminals instead.

Terminal 1:

```bash
source /opt/ros/humble/setup.bash
source ~/project/d1_rl_sar/rl_sar/install/setup.bash
ros2 run dm_imu dm_imu_node --ros-args -p port:=/dev/ttyACM0 -p baud:=921600
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source ~/project/d1_rl_sar/rl_sar/install/setup.bash
ros2 run rl_sar rl_real_d1
```

### Keyboard Control

Common keys:

| Key | FSM transition |
|---|---|
| `0` | Passive / GetDown -> GetUp |
| `1` | GetUp / RLLocomotion -> RLLocomotion |
| `9` | GetUp / RLLocomotion -> GetDown |
| `P` | Any state -> Passive |

### Gamepad Control

The program reads `/dev/input/js0`.

```bash
ls -l /dev/input/js*
ls -l /dev/input/by-id/
```

Add the user to the `input` group if needed:

```bash
sudo usermod -aG input $USER
```

Common transitions:

| Action | FSM transition |
|---|---|
| `A` | Passive -> GetUp |
| `B` | GetUp / RLLocomotion -> GetDown |
| `LB + X` | Any state -> Passive |
| `RB + DPadUp` | GetUp / RLLocomotion -> RLLocomotion |

If no gamepad exists, this message is expected and keyboard control still works:

```text
Joystick [/dev/input/js0] open failed.
```

### Optional MuJoCo Check

MuJoCo is optional for simulation and real-to-sim mapping checks.

Download MuJoCo:

```bash
cd ~/project/d1_rl_sar/rl_sar
bash scripts/download_mujoco.sh
```

Build MuJoCo target:

```bash
cd ~/project/d1_rl_sar/rl_sar
./build.sh -mj
```

Run default scene:

```bash
./cmake_build/bin/rl_sim_mujoco d1 scene
```

Run terrain scene:

```bash
./cmake_build/bin/rl_sim_mujoco d1 scene_terrain
```

Real-to-MuJoCo mapping check:

```bash
source /opt/ros/humble/setup.bash
source ~/project/d1_rl_sar/rl_sar/install/setup.bash
ros2 launch rl_sar rl_real2mujoco.launch.py
```

Manual mode:

```bash
ros2 run dm_imu dm_imu_node --ros-args -p port:=/dev/ttyACM0 -p baud:=921600
ros2 run rl_sar rl_real2mujoco d1 scene
```

Use this to verify IMU direction, motor direction, and CAN ID to joint order.

### Minimal Startup Checklist

Precondition: CANFD autoload is installed and the ROS2 workspace is built.

1. Suspend the robot before power-on tests.
2. Connect IMU, USB-CANFD, and gamepad.
3. Check CAN:

```bash
ip -d link show can1
ip -d link show can2
systemctl --no-pager status d1-gs-usb.service 'd1-canfd@can1.service' 'd1-canfd@can2.service'
```

4. Start:

```bash
source /opt/ros/humble/setup.bash
source ~/project/d1_rl_sar/rl_sar/install/setup.bash
ros2 launch rl_sar rl_real_d1.launch.py
```

5. Press `0` for GetUp.
6. Press `1` for RLLocomotion only after the robot is stable.
7. Press `P` or `LB + X` to return to Passive when anything looks wrong.

### Troubleshooting

#### `ros2 pkg executables rl_sar` does not show `rl_real_d1`

```bash
cd ~/project/d1_rl_sar/rl_sar
source /opt/ros/humble/setup.bash
./build.sh
source install/setup.bash
ros2 pkg executables rl_sar
```

#### `d1_description` is missing from `colcon list`

```bash
ls src/rl_sar_zoo/d1_description/package.xml
colcon list --base-paths src | grep d1_description
```

#### `can1` / `can2` do not exist

```bash
lsusb
lsmod | grep gs_usb
sudo dmesg | grep -i -E 'gs_usb|can|ttyACM' | tail -n 80
```

Check SocketCAN firmware and re-plug the adapter.

#### `d1-gs-usb.service` fails after kernel upgrade

```bash
cd ~/project/d1_rl_sar/5.15
make clean && make
sudo systemctl restart d1-gs-usb.service
```

#### Keyboard does not work under launch

Install `xterm` and use a GUI/X11 session:

```bash
sudo apt install -y xterm
```

For pure SSH without X11, run IMU and `rl_real_d1` in two terminals.

### References

- Upstream RL-SAR: <https://github.com/fan-ziqi/rl_sar>
- Gitee repository: <https://gitee.com/lookc4/d1_rl_sar>
