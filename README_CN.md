# D1 Jetson Orin USB-CANFD 真机部署 README

[English version](README.md)


这是 D1 真机部署说明，目标平台是 Jetson Orin Nano Super + Ubuntu 22.04 + ROS 2 Humble，通信方案是达妙 `DM-USB2CANFD_Dual` 双路 USB-CAN FD，IMU 使用 `DM-IMU-L1`。

本文已按当前仓库状态勘误：项目已经统一改名为 `d1`，不再需要手工把源码里的旧机器人名、CAN 接口、Gazebo 依赖或 ROS2 `package.xml` 做二次 patch。

### 当前验证状态

已验证：

- ROS 2 Humble 可正常构建工作区。
- `d1_description` 已包含有效 ROS2 `package.xml`。
- `rl_sar` 可生成真机入口 `rl_real_d1`。
- `dm_imu_node` 可发布 `/imu/data`。
- `DM-USB2CANFD_Dual` 刷为 SocketCAN 固件后可枚举为 `can1` / `can2`。
- `5.15/gs_usb.ko` 可用于 Jetson 5.15 内核下的 CAN FD 通信。
- `d1-gs-usb.service`、`d1-canfd@.service` 和 udev 规则可自动加载驱动并配置 CAN FD。
- `rl_real_d1` 可启动，FSM 可切换，ONNX 模型可加载。

仍需真机安全验证：

- 接入真实电机后的完整 CAN 收发。
- 机器人吊起状态下的站立、下蹲、行走。

### 硬件和系统要求

| 项目 | 要求 |
|---|---|
| 开发板 | Jetson Orin Nano Super |
| 系统 | Ubuntu 22.04 |
| ROS | ROS 2 Humble |
| 内核 | Jetson 常见 5.15.x |
| 机器人 | D1 |
| IMU | DM-IMU-L1 |
| CAN | DM-USB2CANFD_Dual |
| 推理 | ONNX Runtime，优先 CPU 跑通 |

如果内核不是 5.15.x，`5.15/gs_usb.ko` 需要重新确认兼容性；如果设备不是 `DM-USB2CANFD_Dual`，CAN 配置不能直接照抄。

### 仓库路径

本文假设仓库位于：

```bash
~/project/d1_rl_sar
```

克隆：

```bash
mkdir -p ~/project
cd ~/project
git clone --recursive https://gitee.com/lookc4/d1_rl_sar.git
cd ~/project/d1_rl_sar/rl_sar
```

### 安装基础依赖

先安装 ROS 2 Humble 基础环境。已经装过 ROS 源时执行：

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y ros-humble-ros-base ros-dev-tools
source /opt/ros/humble/setup.bash
```

安装项目依赖：

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

### 下载推理运行库

仓库不提交 `rl_sar/library/` 里的大体积运行库。如果本地没有 ONNX Runtime，先下载：

```bash
cd ~/project/d1_rl_sar/rl_sar
bash scripts/download_inference_runtime.sh onnx
```

### ROS2 编译

普通 ROS2 编译：

```bash
cd ~/project/d1_rl_sar/rl_sar
source /opt/ros/humble/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=1
./build.sh
```

编译后检查：

```bash
source install/setup.bash
colcon list --base-paths src
ros2 pkg executables rl_sar
ros2 pkg executables dm_imu
ldd install/lib/rl_sar/rl_real_d1 | grep 'not found' || true
```

期望看到：

```text
d1_description
rl_sar rl_real_d1
dm_imu dm_imu_node
```

`ldd ... | grep 'not found'` 没有输出表示动态库完整。

### 配置 shell 自动 source

编译成功后，可以把 ROS 和项目 setup 加入 `~/.bashrc`：

```bash
cat >> ~/.bashrc <<'EOF'

source /opt/ros/humble/setup.bash
if [ -f "$HOME/project/d1_rl_sar/rl_sar/install/setup.bash" ]; then
  source "$HOME/project/d1_rl_sar/rl_sar/install/setup.bash"
fi
EOF
```

当前终端立即生效：

```bash
source ~/.bashrc
```

### IMU 验证

串口权限：

```bash
groups
sudo usermod -aG dialout $USER
newgrp dialout
```

启动 IMU，实际验证串口为 `/dev/ttyACM0`：

```bash
source /opt/ros/humble/setup.bash
source ~/project/d1_rl_sar/rl_sar/install/setup.bash
ros2 run dm_imu dm_imu_node --ros-args -p port:=/dev/ttyACM0 -p baud:=921600
```

另开终端检查：

```bash
source /opt/ros/humble/setup.bash
source ~/project/d1_rl_sar/rl_sar/install/setup.bash
ros2 topic echo /imu/data --once
```

看到 `sensor_msgs/msg/Imu` 即为正常。

### USB-CANFD 固件

`DM-USB2CANFD_Dual` 出厂固件通常不是 Linux SocketCAN 模式。使用 SocketCAN 前，需要用达妙官方升级工具刷入 socketcan 固件。

参考固件名：

```text
dm_usb2canfd_dual_gsusb_1004.enc
```

刷入前常见现象：

- 设备可能表现为 `ttyACM*`。
- `lsusb` 可能显示 `DaMiao-Tech DM-USB2FDCAN`。

刷入 socketcan 固件并重新插拔后，正常现象：

- `lsusb` 显示 `1d50:606f OpenMoko ... CAN adapter`。
- Linux 下出现 CAN 网卡，Jetson 板载 CAN 占用 `can0` 时，USB 双路通常是 `can1` 和 `can2`。

### 编译并加载 gs_usb 驱动

Jetson 5.15 内核下，CAN FD 需要使用仓库 `5.15/` 里的自定义 `gs_usb` 驱动。

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

再次 `insmod` 如果提示 `File exists`，说明模块已经加载，不是故障。

### 推荐：开机自动加载 CANFD

仓库已经提供自动加载脚本：

```bash
cd ~/project/d1_rl_sar/5.15/autoload
sudo bash install.sh
```

脚本会安装：

| 文件 | 作用 |
|---|---|
| `/etc/modules-load.d/d1-can.conf` | 开机加载 `can`、`can_raw`、`can_dev` |
| `/etc/systemd/system/d1-gs-usb.service` | 开机加载当前路径下的 `gs_usb.ko` |
| `/etc/udev/rules.d/85-d1-canfd.rules` | 监听 `gs_usb` 生成的 CAN 网卡 |
| `/etc/systemd/system/d1-canfd@.service` | 自动配置 `1M arbitration / 5M data / FD on` 并拉起接口 |

检查状态：

```bash
systemctl --no-pager status d1-gs-usb.service
systemctl --no-pager status 'd1-canfd@can1.service' 'd1-canfd@can2.service'
ip -d link show can1
ip -d link show can2
```

期望：

- `d1-gs-usb.service` 为 `active (exited)`。
- `can1` / `can2` 已配置 `bitrate 1000000`、`dbitrate 5000000`、`fd on`。
- 未接真实设备时，接口可能显示 `NO-CARRIER`，这不一定是故障。

卸载自动加载：

```bash
sudo bash ~/project/d1_rl_sar/5.15/autoload/uninstall.sh
```

如果 Jetson 内核升级，必须重新编译 `gs_usb.ko`：

```bash
cd ~/project/d1_rl_sar/5.15
make clean && make
sudo systemctl restart d1-gs-usb.service
```

### 手动配置 CANFD

如果没有安装自动加载，手动执行：

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

### CANFD 回环测试

在未接电机前，建议先做双通道外部回环。按硬件说明把 `can1` 和 `can2` 正确连接后执行：

终端 1：

```bash
sudo candump can2 -t a
```

终端 2：

```bash
sudo cansend can1 123##91122334455667788
```

`candump` 能收到报文，说明固件、驱动和双路 CAN FD 链路基本正常。

### CAN 接口约定

当前 `rl_real_d1.cpp` 已经按 Jetson 实机部署约定写成：

| 逻辑总线 | SocketCAN 接口 | 用途 |
|---|---|---|
| CAN1 | `can1` | 第一组 6 个电机 |
| CAN2 | `can2` | 第二组 6 个电机 |

不需要再手工把 `can0/can1` 改成 `can1/can2`。

### 电机零点标定

D1 使用达妙 `DM6248P` 电机。零点写入每颗电机自己的 FLASH，断电不丢失。首次部署、换电机或机械重装后必须标定。

前置条件：

- `can1` / `can2` 已经 `UP`。
- 每条总线的电机 CAN ID 已设置为 `0x01..0x06`。
- 机器人吊起，四条腿可自由活动。
- 12 个关节摆到 URDF 零位。
- `rl_real_d1`、`test_motor` 等会占用 CAN 的程序都没有运行。

全车标定：

```bash
source /opt/ros/humble/setup.bash
source ~/project/d1_rl_sar/rl_sar/install/setup.bash
ros2 run dmbot_serial set_zero_all
```

单电机补标：

```bash
ros2 run dmbot_serial set_zero_one can1 0x02
ros2 run dmbot_serial set_zero_one can2 5
```

CAN ID 对应关系：

| can1 ID | 关节 | can2 ID | 关节 |
|---|---|---|---|
| 0x01 | FL_hip | 0x01 | RL_hip |
| 0x02 | FL_thigh | 0x02 | RL_thigh |
| 0x03 | FL_calf | 0x03 | RL_calf |
| 0x04 | FR_hip | 0x04 | RR_hip |
| 0x05 | FR_thigh | 0x05 | RR_thigh |
| 0x06 | FR_calf | 0x06 | RR_calf |

写入 FLASH 后建议给电机物理断电再上电，然后重新跑 `set_zero_all`，只看 BEFORE 表格是否全部接近 0。

### 真机运行

安全要求：首次运行必须吊起机器人，确认急停方式可用，再进入站立或行走状态。

一键 launch，适合有图形环境的 Jetson：

```bash
source /opt/ros/humble/setup.bash
source ~/project/d1_rl_sar/rl_sar/install/setup.bash
ros2 launch rl_sar rl_real_d1.launch.py
```

`rl_real_d1.launch.py` 会同时启动：

- `dm_imu_node`
- `rl_real_d1`

默认 IMU 参数：

```text
port=/dev/ttyACM0
baud=921600
```

覆盖 IMU 串口：

```bash
ros2 launch rl_sar rl_real_d1.launch.py port:=/dev/ttyACM1 baud:=921600
```

注意：launch 会用 `xterm -hold -e` 包住 `rl_real_d1`，因为键盘控制需要真实 TTY。纯 SSH 无 X11 时，改用两个终端分别启动。

终端 1：

```bash
source /opt/ros/humble/setup.bash
source ~/project/d1_rl_sar/rl_sar/install/setup.bash
ros2 run dm_imu dm_imu_node --ros-args -p port:=/dev/ttyACM0 -p baud:=921600
```

终端 2：

```bash
source /opt/ros/humble/setup.bash
source ~/project/d1_rl_sar/rl_sar/install/setup.bash
ros2 run rl_sar rl_real_d1
```

### 键盘控制

`rl_real_d1` 的键盘输入需要 TTY。常用按键：

| 按键 | FSM 切换 |
|---|---|
| `0` | Passive / GetDown -> GetUp |
| `1` | GetUp / RLLocomotion -> RLLocomotion |
| `9` | GetUp / RLLocomotion -> GetDown |
| `P` | 任意状态 -> Passive |

### 手柄控制

程序硬编码读取 Linux joystick 设备 `/dev/input/js0`。检查：

```bash
ls -l /dev/input/js*
ls -l /dev/input/by-id/
```

建议把用户加入 `input` 组：

```bash
sudo usermod -aG input $USER
# 重新登录后确认 groups 包含 input
```

常用手柄切换：

| 操作 | FSM 切换 |
|---|---|
| `A` | Passive -> GetUp |
| `B` | GetUp / RLLocomotion -> GetDown |
| `LB + X` | 任意状态 -> Passive |
| `RB + DPadUp` | GetUp / RLLocomotion -> RLLocomotion |

没有手柄时，程序会打印：

```text
Joystick [/dev/input/js0] open failed.
```

这不是阻塞项，键盘控制仍可使用。

### MuJoCo 可选校验

MuJoCo 用于仿真或真机映射检查，不是实机最短部署链路的一部分。

下载 MuJoCo：

```bash
cd ~/project/d1_rl_sar/rl_sar
bash scripts/download_mujoco.sh
```

编译 MuJoCo：

```bash
cd ~/project/d1_rl_sar/rl_sar
./build.sh -mj
```

运行普通场景：

```bash
./cmake_build/bin/rl_sim_mujoco d1 scene
```

运行地形场景：

```bash
./cmake_build/bin/rl_sim_mujoco d1 scene_terrain
```

真机到 MuJoCo 的映射检查：

```bash
source /opt/ros/humble/setup.bash
source ~/project/d1_rl_sar/rl_sar/install/setup.bash
ros2 launch rl_sar rl_real2mujoco.launch.py
```

手动分开启动：

```bash
ros2 run dm_imu dm_imu_node --ros-args -p port:=/dev/ttyACM0 -p baud:=921600
ros2 run rl_sar rl_real2mujoco d1 scene
```

用于检查：

- IMU roll / pitch / yaw 方向是否和 MuJoCo 一致。
- 每个电机方向是否同向同幅度。
- CAN ID 到 FL / FR / RL / RR 的关节顺序是否正确。

### 最短启动清单

前提：已经安装 CANFD 自动加载，且工作区已经编译。

1. 上电前吊起机器人。
2. 插好 IMU、USB-CANFD 和手柄。
3. 检查 CAN：

```bash
ip -d link show can1
ip -d link show can2
systemctl --no-pager status d1-gs-usb.service 'd1-canfd@can1.service' 'd1-canfd@can2.service'
```

4. 启动：

```bash
source /opt/ros/humble/setup.bash
source ~/project/d1_rl_sar/rl_sar/install/setup.bash
ros2 launch rl_sar rl_real_d1.launch.py
```

5. 先按 `0` 进入 GetUp。
6. 确认姿态稳定后再按 `1` 进入 RLLocomotion。
7. 异常时按 `P` 或 `LB + X` 回 Passive，并断电检查。

### 常见问题

#### `ros2 pkg executables rl_sar` 没有 `rl_real_d1`

重新构建并 source：

```bash
cd ~/project/d1_rl_sar/rl_sar
source /opt/ros/humble/setup.bash
./build.sh
source install/setup.bash
ros2 pkg executables rl_sar
```

#### `d1_description` 不在 `colcon list` 里

确认仓库是最新版本，且存在：

```bash
ls src/rl_sar_zoo/d1_description/package.xml
colcon list --base-paths src | grep d1_description
```

#### `can1` / `can2` 不存在

检查固件、驱动和枚举：

```bash
lsusb
lsmod | grep gs_usb
sudo dmesg | grep -i -E 'gs_usb|can|ttyACM' | tail -n 80
```

确认 USB-CANFD 已刷 socketcan 固件，并重新插拔。

#### `d1-gs-usb.service` 启动失败

常见原因是内核升级后旧 `gs_usb.ko` 不匹配：

```bash
cd ~/project/d1_rl_sar/5.15
make clean && make
sudo systemctl restart d1-gs-usb.service
```

#### launch 后键盘没反应

确认 `xterm` 已安装，并且当前 session 有图形环境：

```bash
sudo apt install -y xterm
```

纯 SSH 无 X11 时使用两个终端分别运行 IMU 和 `rl_real_d1`。

### 参考

- Upstream RL-SAR: <https://github.com/fan-ziqi/rl_sar>
- Gitee repository: <https://gitee.com/lookc4/d1_rl_sar>

