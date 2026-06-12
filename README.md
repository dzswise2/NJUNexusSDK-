# Nexus SDK

Nexus SDK 是一个基于 ROS2 的机器人开发套件，提供机器人仿真、遥操作、数据采集等功能。

当前主线：**Nexus-Arm V15 Right → Franka FR3v2.1 + Hand** 主从遥操作，快速上手见 [Franka 专章](#nexus-arm-v15-right--franka-fr3v21--hand)。

## 功能特性

- **MuJoCo 仿真环境** (`mujoco_sim`): 基于 MuJoCo 物理引擎的机器人仿真，支持多种机器人模型
- **遥操作适配器** (`teleop_adapter`): 遥操作数据转发和处理
- **机器人控制器** (`robot_controller`): 机器人运动控制
- **运动学解算** (`human_data`): 主从臂正/逆运动学与人机状态发布
- **消息定义** (`infra_msg`): ROS2 消息和服务定义
- **夹爪控制** (`gripper_keyboard`): 夹爪键盘控制工具
- **管理工具** (`nexus_manage`): 系统管理和配置工具

## 支持的机器人

| 角色 | 机型 | 仿真 | 实机 |
|------|------|------|------|
| 主臂 | Nexus-Arm V15 Right | ✅ | ✅ |
| 从臂 | Franka FR3v2.1 + Hand | ✅ | 需实现 `franka_hand` adapter |

> 工作区内亦保留 AR5、Piper 等历史配置，**本文示例均以 V15 → Franka 为准**。  
> - 调参详解：[`docs/franka_tuning_guide.md`](docs/franka_tuning_guide.md)  
> - 实机 `teleop_adapter`：[`docs/franka_teleop_adapter_real_deployment.md`](docs/franka_teleop_adapter_real_deployment.md)

---

## 环境要求

- **操作系统**: Ubuntu 22.04
- **ROS2**: Humble
- **Python**: 3.10+
- **MuJoCo**: 3.0+

---

## 仓库拉取

### 1. 配置 SSH 密钥（推荐）

```bash
# 生成 SSH 密钥（如果没有）
ssh-keygen -t ed25519 -C "your.email@example.com"

# 将公钥添加到 GitLab
cat ~/.ssh/id_ed25519.pub
# 复制输出内容，添加到 GitLab -> Settings -> SSH Keys
```

### 2. 克隆仓库

**使用 SSH（推荐）:**
```bash
git clone git@github.com:dzswise2/NJUNexusSDK-.git
cd NJUNexusSDK-
```

**使用 HTTPS:**
```bash
git clone https://github.com/dzswise2/NJUNexusSDK-.git
cd NJUNexusSDK-
```

### 3. 初始化子模块

```bash
git submodule update --init --recursive
```

---

## 依赖安装

### 1. 安装 ROS2 Humble

如果尚未安装 ROS2，请参考 [ROS2 Humble 官方安装指南](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)。

```bash
# 快速安装命令
sudo apt update && sudo apt install -y ros-humble-desktop
```

### 2. 安装系统依赖

```bash
# 基础构建工具
sudo apt install -y build-essential cmake pkg-config

# Eigen3（robot_controller、human_data、nexus_manage 依赖）
sudo apt install -y libeigen3-dev

# yaml-cpp（nexus_manage、vision_data_hub 依赖）
sudo apt install -y libyaml-cpp-dev

# libevdev（human_data、gripper_keyboard 依赖，用于输入设备读取）
sudo apt install -y libevdev-dev

# Pinocchio 机器人动力学库（robot_controller、human_data 依赖）
sudo apt install -y ros-humble-pinocchio

# nlohmann-json（vision_data_hub 依赖）
sudo apt install -y nlohmann-json3-dev

# OpenCV（vision_data_hub、data_collector 依赖）
sudo apt install -y libopencv-dev
```

### 3. 安装视觉相关依赖（vision_data_hub 需要）

```bash
# RealSense SDK
# 参考: https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md
sudo apt install -y librealsense2-dev librealsense2-utils

# FFmpeg 编解码
sudo apt install -y libavcodec-dev libavutil-dev libavformat-dev libswscale-dev

# GStreamer（WebRTC 推流）
sudo apt install -y \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad gstreamer1.0-nice

# libsoup（WebSocket 信令）
sudo apt install -y libsoup2.4-dev
```

### 4. 安装 OSQP 和 OsqpEigen（robot_controller 需要）

`robot_controller` 使用 OSQP 求解器进行二次规划，需要从源码编译安装：

```bash
# 编译安装 OSQP
cd ~
git clone --branch release-0.6.3 --recursive https://github.com/osqp/osqp.git
cd osqp && mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=~/osqp/install ..
make -j$(nproc) && make install
cd ~

# 编译安装 OsqpEigen
git clone https://github.com/robotology/osqp-eigen.git
cd osqp-eigen && mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=~/osqp-eigen/install -DCMAKE_PREFIX_PATH=~/osqp/install ..
make -j$(nproc) && make install
cd ~
```

> **提示**: 默认安装路径为 `~/osqp/install` 和 `~/osqp-eigen/install`。如需自定义路径，编译时设置环境变量：
> ```bash
> export OSQP_INSTALL_DIR=/your/custom/path/osqp/install
> export OSQP_EIGEN_INSTALL_DIR=/your/custom/path/osqp-eigen/install
> ```

### 5. 安装 MuJoCo

```bash
pip3 install mujoco
```

### 5.1 安装 libfranka（实机从臂，可选）

实机 Slave 侧 `teleop_adapter` 需 libfranka，版本须与 Franka 固件匹配，按 [Franka 官方文档](https://frankaemika.github.io/docs/libfranka.html) 安装后再编译 `teleop_adapter`。

### 6. 安装 Python 依赖（仿真）

```bash
# 基础依赖（mujoco_sim 需要）
pip3 install numpy pyyaml

# data_collector 依赖（数据采集功能需要）
pip3 install pyarrow pandas av "opencv-python>=4.8.0,<4.10.0"

# 或者通过 requirements.txt 安装
pip3 install -r src/data_collector/requirements.txt
```

### 7. 安装 ROS2 依赖

```bash
# Source ROS2 环境
source /opt/ros/humble/setup.bash

# 进入工作空间
cd NJUNexusSDK-

# 使用 rosdep 安装依赖
sudo apt install -y python3-rosdep
sudo rosdep init  # 如果之前没有初始化过
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

---

## 编译

### 1. Source ROS2 环境

```bash
source /opt/ros/humble/setup.bash
```

### 2. 编译整个工作空间

```bash
cd NJUNexusSDK-
colcon build --symlink-install
```

### 3. 编译特定包（Franka 仿真 / 实机）

```bash
# Franka 全系统仿真（推荐首次验证）
colcon build --symlink-install --packages-select infra_msg
source install/setup.bash
colcon build --symlink-install --packages-up-to nexus_manage

# 仅改 controller 参数后
colcon build --symlink-install --packages-select robot_controller nexus_manage

# 实机 adapter 开发后
colcon build --symlink-install --packages-select teleop_adapter
```

### 4. Source 工作空间

```bash
source install/setup.bash
```

> **提示**: 建议将以下内容添加到 `~/.bashrc`:
> ```bash
> source /opt/ros/humble/setup.bash
> source ~/NJUNexusSDK-/install/setup.bash  # 根据实际路径修改
> ```

---

## 快速开始

> **前提**：每次打开新终端时，需要先 source 环境：
> ```bash
> source /opt/ros/humble/setup.bash
> source install/setup.bash
> ```

---

## Nexus-Arm V15 Right → Franka FR3v2.1 + Hand

当前主线：**V15 Right 主臂**遥操作 **Franka FR3v2.1 + Hand 从臂**。仿真由 `mujoco_sim` 驱动从臂；实机由 `teleop_adapter` 经 libfranka/FCI 驱动（adapter 需乙方实现）。

### 1. 仿真启动

#### 1.1 确认仿真模式

从臂与主臂 controller 配置中 `is_simulation` 应为 `true`：

| 文件 | 字段 |
|------|------|
| `robot_controller/config/arm_control_fr3v2_1_hand.yaml` | `master_arm_controller.is_simulation: true` |
| `robot_controller/config/slave_single_fr3v2_1_hand_config.yaml` | `is_simulation: true` |
| `robot_controller/config/master_arm_control_nexus_v15_right.yaml` | `master_arm_controller.is_simulation: true` |

修改 yaml 后重新编译并 source：

```bash
colcon build --symlink-install --packages-select robot_controller
source install/setup.bash
```

#### 1.2 一键启动全系统仿真

```bash
ros2 launch nexus_manage nexus_arm_v15_right_to_fr3v2_1_hand_sim_system.launch.py
```

自动启动：主臂/从臂 MuJoCo 仿真、`human_data`、`robot_controller`（主从）、`nexus_manage`、`gripper_keyboard`。

仅调试 Franka 从臂仿真：

```bash
ros2 launch mujoco_sim fr3v2_1_hand_sim.launch.py
```

#### 1.3 遥操作步骤

1. 等待自检通过（约 5 s）
2. 小键盘 `1` → 主从复位
3. 按住 `R`，长按 `Q` 3 s → 进入遥操作
4. 拖动主臂，从臂跟随；松开 `R` → 位置保持

键盘说明见本文 [键盘按键说明](#键盘按键说明) 与 [遥操作使用流程](#遥操作使用流程)。

---

### 2. 调参说明

从臂核心参数在：

- **`robot_controller/config/arm_control_fr3v2_1_hand.yaml`** — 7 轴 IK、MIT 阻抗、安全域、外力估计
- **`robot_controller/config/gripper_config_fr3v2_1_hand.yaml`** — `finger_joint1` 夹爪（m）

完整分节说明、默认值、调参顺序与「现象 → 参数」对照表见：

**[`docs/franka_tuning_guide.md`](docs/franka_tuning_guide.md)**

快速索引：

| 想调什么 | 优先改哪里 |
|----------|------------|
| 复位姿态 / 夹爪全开对齐 | `nexus_manage/.../nexus-arm_v15_right_to_fr3v2_1_hand_manage.yaml` |
| 跟手 / 抖动 | `vqp_clik_gain`、`motion_planning_kp/kd`、`vqp_max_total_joint_change` |
| 肘部漂移 / 奇异 | `vqp_arm_angle_*`、`vqp_ref_joint_*` |
| 夹爪行程比例 | `gripper_scaling_factor`（manage） |
| 夹爪闭合速度 / 刚度 | `gripper_config_fr3v2_1_hand.yaml` 的 `kp`、`max_gripper_delta` |
| 主臂力反馈 | `master_arm_control_nexus_v15_right.yaml` 的 `ee_ff_gain` |

修改 yaml 后执行 `colcon build --symlink-install --packages-select robot_controller nexus_manage` 并 `source install/setup.bash`。

---

### 3. Franka 实机 teleop_adapter 适配（概要）

仿真阶段 **不启动** `teleop_adapter`；实机从臂必须由 `teleop_adapter` 订阅 `/fr3v2_1/robot/joint_cmd` 并经 libfranka 驱动硬件。

**乙方需完成：**

1. 新增 `FrankaArmAdapter`（`franka_arm_adapter.hpp/.cpp`），继承 `DeviceAdapter`
2. 在 `teleop_arm_driver_node.cpp` 的 `createAdapter()` 注册 `adapter_type: franka_hand`
3. `CMakeLists.txt` 链接 libfranka
4. 7 轴 MIT 力矩环（~1000 Hz）+ 夹爪 `finger_joint1` 0～0.04 m（独立线程，勿阻塞臂回调）

**仓库已有骨架（无需新建 launch/yaml 路径）：**

- `teleop_adapter/config/slave_fr3v2_1_hand_single.yaml`
- `teleop_adapter/launch/slave_fr3v2_1_hand_single.launch.py`

**不可复用** AR5 adapter；可参考 `ar5_arm_adapter.cpp` 的三线程架构。

完整实现步骤、接口约定、分阶段验收与故障排查 → **[`docs/franka_teleop_adapter_real_deployment.md`](docs/franka_teleop_adapter_real_deployment.md)**

---

### 4. 实机部署与启动

> 前提：已完成 §3 的 `franka_hand` adapter 实现并通过单节点验收。

#### 4.1 实机配置检查

| 项 | 操作 |
|----|------|
| FCI 网络 | 修改 `slave_fr3v2_1_hand_single.yaml` 中 `right_arm_port` / `right_arm_local_ip` |
| 仿真标志 | `slave_single_fr3v2_1_hand_config.yaml` → `is_simulation: false` |
| 主臂实机 | `master_arm_control_nexus_v15_right.yaml` → `is_simulation: false` |
| 臂控标定 | 按 FR3 修改 `arm_control_fr3v2_1_hand.yaml` 增益与限位 |

#### 4.2 编译

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select infra_msg
source install/setup.bash
colcon build --symlink-install --packages-up-to nexus_manage
source install/setup.bash
```

#### 4.3 启动方式

**双机同网（推荐）**

```bash
# Slave 工控机（先启动）
export ROS_DOMAIN_ID=18
ros2 launch nexus_manage nexus_arm_v15_right_to_fr3v2_1_hand_real_system.launch.py role:=slave

# Master 机（Nexus V15）
ros2 launch nexus_manage nexus_arm_v15_right_to_fr3v2_1_hand_real_system.launch.py role:=master
```

**单机首检**

```bash
ros2 launch nexus_manage nexus_arm_v15_right_to_fr3v2_1_hand_real_system.launch.py
```

**跨子网**

```bash
# 先改 cross_subnet launch 顶部网络常量 + slave_registry.yaml
ros2 launch nexus_manage nexus_arm_v15_right_to_fr3v2_1_hand_real_system_cross_subnet.launch.py \
  role:=slave robot_id:=fr3v2_1_01
ros2 launch nexus_manage nexus_arm_v15_right_to_fr3v2_1_hand_real_system_cross_subnet.launch.py \
  role:=master
```

#### 4.4 Slave 侧节点（实机无 mujoco_sim）

| 包 | Launch |
|----|--------|
| teleop_adapter | `slave_fr3v2_1_hand_single.launch.py` |
| human_data | `fr3v2_1_hand_human_data_solver.launch.py` |
| robot_controller | `slave_single_fr3v2_1_hand.launch.py` |

#### 4.5 验收 Topic

```bash
ros2 topic hz /fr3v2_1/robot/joint_states
ros2 topic list | grep -E 'fr3v2_1|teleop|robot'
```

遥操作步骤同 §1.3。

---

### 5. 系统架构与节点

```
┌──────────────────┐     ┌──────────────┐     ┌──────────────┐     ┌─────────────────────┐
│ Nexus-Arm V15    │────▶│ human_data   │────▶│ nexus_manage │────▶│ Franka FR3v2.1+Hand │
│ Right（主臂）     │     │ (运动学解算)  │     │ (遥操作管理)  │     │ （从臂）             │
└──────────────────┘     └──────────────┘     └──────────────┘     └─────────────────────┘
        │                                              │                        │
        ▼                                              ▼                        ▼
 robot_controller                              gripper_keyboard          robot_controller
   (主臂控制器)                                    (夹爪控制)                (从臂控制器)
                                                                                  │
                                    仿真: mujoco_sim ◀── joint_cmd ─────────────┘
                                    实机: teleop_adapter (franka_hand) ◀── joint_cmd
```

**仿真一键启动**（§1.2）会自动拉起：

| 节点 | 包 | 说明 |
|------|-----|------|
| V15 Right 仿真 | `mujoco_sim` | 主臂 MuJoCo |
| Franka+Hand 仿真 | `mujoco_sim` | 从臂 MuJoCo（`fr3v2_1_hand`） |
| 运动学解算 | `human_data` | 主从 FK |
| 主臂控制器 | `robot_controller` | `master_single_nexus_v15_right` |
| 从臂控制器 | `robot_controller` | `slave_single_fr3v2_1_hand` |
| 遥操作管理 | `nexus_manage` | 状态机、主从映射 |
| 夹爪键盘 | `gripper_keyboard` | 脚踏/遥操按键 |

---

### 键盘按键说明

系统通过 `libevdev` 直接从 `/dev/input/event*` 设备读取键盘输入（而非终端 stdin），因此**按键在任何窗口聚焦状态下都有效**。按键由两个节点分别读取：

> ⚠️ **前提**：用户必须在 `input` 组中才能读取键盘设备。首次使用需执行：
> ```bash
> sudo bash tool/setup_pedal_permissions.sh
> # 然后注销重新登录，或在当前终端执行: newgrp input
> ```

#### 脚踏板按键（`human_data` 节点读取，小键盘）

| 键盘按键 | 逻辑名称 | 功能 |
|---------|---------|------|
| 小键盘 `1` | reset_pedal | 触发复位流程 |
| 小键盘 `2` | suspend_pedal | 暂停/恢复遥操作 |
| 小键盘 `3` | increment_pedal | 切换计算模式（Scaling ↔ 增量） |
| 小键盘 `4` | scaling_plus_pedal | 增大缩放系数 |
| 小键盘 `5` | scaling_minus_pedal | 减小缩放系数 |

#### 夹爪/控制按键（`gripper_keyboard` 节点读取，主键盘 QWERTY）

| 键盘按键 | 逻辑名称 | 功能 |
|---------|---------|------|
| `Q` | teleop_key | 遥操按键（长按 3 秒启动遥操） |
| `W` | data_collect_key | 数据采集按键 |
| `E` | marker_key | 数据标记按键 |
| `R` | safety_key | 安全按键（遥操运行中必须按住） |
| `T` | take_over_key | 接管按键（切换到模型推理模式） |

### 遥操作使用流程

系统启动后，`nexus_manage` 管理器通过状态机控制整个遥操作流程。

#### 步骤一：等待自检完成

系统启动后自动进入 **BootSelfCheck**（启动自检）状态，检查所有节点是否就绪（约 5 秒）。自检通过后自动进入 **Idle**（空闲）状态。

> 如果自检超时失败，系统会进入 **Fault** 状态，请检查各节点日志排查问题。

#### 步骤二：复位机械臂

在 Idle 状态下，**按下小键盘 `1`**（reset），系统进入 **Reset**（复位）状态，主臂和从臂将运动到预设的初始位置。复位完成后进入 **ResetComplete**（复位完成）状态。

#### 步骤三：启动遥操作

在 ResetComplete 状态下，**按住 `R`（safety）的同时长按 `Q`（teleop）3 秒**，系统进入 **TeleopRunning**（遥操运行）状态。此时移动主臂，从臂会同步跟随运动。

> ⚠️ **安全机制**：遥操运行中必须始终按住 `R`（safety）。**松开 `R`** 会立即进入 **PositionHold**（位置保持），从臂停止在当前位置。

#### Franka 仿真遥操作快速示例

```
1. 启动系统:  ros2 launch nexus_manage nexus_arm_v15_right_to_fr3v2_1_hand_sim_system.launch.py
2. 等待自检通过 (观察终端日志，约 5 秒)
3. 按 小键盘1     → 复位 (Reset → ResetComplete)，主从夹爪应对齐全开
4. 按住 R，长按 Q 3秒 → 启动遥操 (TeleopRunning)
5. 拖动主臂，Franka 仿真从臂同步跟随；观察 finger_joint1 随主臂夹爪变化
6. 松开 R          → 位置保持 (PositionHold)
7. 按 小键盘1     → 重新复位
```

#### 遥操作中的控制

| 操作 | 按键 | 效果 |
|------|------|------|
| 松开安全按键 | 松开 `R` | 进入 PositionHold，从臂保持当前位置 |
| 暂停遥操 | 小键盘 `2` 按下 | 进入 TeleopPaused，松开后自动恢复 |
| 切换模型推理 | 按 `T` | 在 TeleopRunning 下进入 ModelInference |
| 重新复位 | 在 PositionHold 下按小键盘 `1` | 回到 Reset 状态重新复位 |

#### 状态流转图

```
BootSelfCheck ──自检通过──▶ Idle ──小键盘1──▶ Reset ──复位完成──▶ ResetComplete
                                                                       │
                        ┌─────────────────────── Q长按3s + R按住 ──────┘
                        ▼
                  TeleopRunning ◀──小键盘2松开── TeleopPaused
                        │    │
                        │    └──小键盘2按下──▶ TeleopPaused
                        │
                        └──R松开──▶ PositionHold ──小键盘1──▶ Reset
```

### 单独启动仿真（调试）

```bash
# Franka FR3v2.1 + Hand 从臂
ros2 launch mujoco_sim fr3v2_1_hand_sim.launch.py

# Nexus-Arm V15 Right 主臂
ros2 launch mujoco_sim nexus_arm_v15_right_sim.launch.py
```

从臂关键 topic（`robot_name=fr3v2_1`）：

```bash
ros2 topic echo /fr3v2_1/robot/joint_states
ros2 topic hz /fr3v2_1/robot/joint_cmd
```

---

## 项目结构（Franka 链路）

```
NJUNexusSDK-/
├── docs/
│   ├── franka_tuning_guide.md                     # 从臂调参详解
│   └── franka_teleop_adapter_real_deployment.md   # 实机 adapter 详细说明
├── src/
│   ├── nexus_manage/
│   │   ├── launch/nexus_arm_v15_right_to_fr3v2_1_hand_*   # 仿真/实机系统 launch
│   │   └── config/nexus-arm_v15_right_to_fr3v2_1_hand_manage.yaml
│   ├── mujoco_sim/
│   │   ├── launch/fr3v2_1_hand_sim.launch.py
│   │   ├── config/fr3v2_1_hand_mujoco_sim.yaml
│   │   └── robot_description/fr3v2_1_franka_hand/
│   ├── robot_controller/
│   │   └── config/arm_control_fr3v2_1_hand.yaml           # 从臂调参主文件
│   ├── human_data/
│   │   └── config/fr3v2_1_hand_human_data_config.yaml
│   └── teleop_adapter/
│       ├── config/slave_fr3v2_1_hand_single.yaml          # 实机 FCI 配置
│       └── launch/slave_fr3v2_1_hand_single.launch.py
└── ...
```

---

## Franka 关键 Launch 速查

| 场景 | 命令 |
|------|------|
| 仿真全系统 | `ros2 launch nexus_manage nexus_arm_v15_right_to_fr3v2_1_hand_sim_system.launch.py` |
| 实机双机 | `..._real_system.launch.py role:=master` / `role:=slave` |
| 实机单机 | `..._real_system.launch.py` |
| 跨子网 | `..._real_system_cross_subnet.launch.py` + `robot_id:=fr3v2_1_01` |
| 仅从臂仿真 | `ros2 launch mujoco_sim fr3v2_1_hand_sim.launch.py` |
| 仅从臂 teleop | `ros2 launch teleop_adapter slave_fr3v2_1_hand_single.launch.py` |

---

## 常见问题

### Q: 编译时报错找不到 infra_msg

**A**: 先编译 `infra_msg`，再编 Franka 相关包：
```bash
colcon build --symlink-install --packages-select infra_msg
source install/setup.bash
colcon build --symlink-install --packages-up-to nexus_manage
```

### Q: 仿真从臂不动 / 自检失败

**A**: 确认 `arm_control_fr3v2_1_hand.yaml` 与 `slave_single_fr3v2_1_hand_config.yaml` 中 `is_simulation: true`，并重新编译 `robot_controller`。

### Q: 实机报 `Unsupported adapter_type: franka_hand`

**A**: `teleop_adapter` 尚未实现 `FrankaArmAdapter`，见 [`docs/franka_teleop_adapter_real_deployment.md`](docs/franka_teleop_adapter_real_deployment.md)。

### Q: 主从夹爪行程不一致

**A**: 检查 `nexus-arm_v15_right_to_fr3v2_1_hand_manage.yaml` 的 `gripper_scaling_factor` 与复位 `robot_pos_values: [0.04]`；运行 `derive_franka_gripper_mapping.py` 重算。

### Q: MuJoCo 窗口无法显示

**A**: 检查图形环境，或在 `fr3v2_1_hand_mujoco_sim.yaml` 设 `enable_viewer: false`

---

## 贡献指南

1. Fork 本仓库
2. 创建特性分支 (`git checkout -b feature/amazing-feature`)
3. 提交更改 (`git commit -m 'Add some amazing feature'`)
4. 推送到分支 (`git push origin feature/amazing-feature`)
5. 创建 Merge Request

---

## 许可证

本项目为内部项目，仅供公司内部使用。

---

## 联系方式

如有问题，请联系：
- GitHub Issues: [NJUNexusSDK- Issues](https://github.com/dzswise2/NJUNexusSDK-/issues)
