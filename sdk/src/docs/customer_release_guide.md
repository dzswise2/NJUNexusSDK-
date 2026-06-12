# Nexus SDK 客户发布包使用指南

本文档面向收到 **Nexus SDK 客户发布包** 的集成方，说明如何部署预编译库、编译 `teleop_adapter` 源码，以及如何适配其他品牌机械臂。

---

## 1. 发布包组成

| 包名 | 分发形式 | 说明 |
|------|----------|------|
| `infra_msg` | 源码 | ROS2 消息/服务定义，需本地编译 |
| `robot_controller` | **预编译库** | 从臂运动控制（闭源） |
| `nexus_manage` | **预编译库** | 主从状态机与路由（闭源） |
| `human_data` | **预编译库** | 主从运动学解算（闭源） |
| `teleop_adapter` | **完整源码** | 硬件 I/O 适配层，**客户自行编译并扩展** |

> **设计意图**：核心控制算法以预编译库形式交付；`teleop_adapter` 保留源码，便于客户接入 Franka、UR、ABB 等不同品牌机械臂的厂商 SDK。

### 1.1 目录结构

解压后的工作空间 `src/` 目录结构如下：

```
src/
├── README.md                 # 本文档副本
├── infra_msg/                # 消息定义（源码）
├── robot_controller/         # 预编译库
│   ├── CMakeLists.txt        #   自动生成的安装脚本
│   ├── package.xml
│   ├── lib/
│   │   ├── x86_64/           #   x86_64 平台 .so / 可执行文件
│   │   └── aarch64/          #   aarch64 平台 .so / 可执行文件
│   ├── include/              #   公开头文件（不含 detail/）
│   ├── launch/
│   ├── config/
│   └── urdf/
├── nexus_manage/             # 预编译库（结构同上）
├── human_data/               # 预编译库（结构同上）
└── teleop_adapter/           # 完整源码
    ├── CMakeLists.txt        #   原始 CMake（非生成文件）
    ├── package.xml
    ├── include/
    ├── src/                  #   .cpp 源码
    ├── launch/
    ├── config/
    ├── external/             #   厂商 SDK 头文件/库（Rokae 等）
    └── docs/
```

CMake 会根据当前 CPU 架构（`x86_64` 或 `aarch64`）自动选择 `lib/${NEXUS_ARCH}/` 下的预编译产物。

---

## 2. 环境要求

| 组件 | 版本 |
|------|------|
| 操作系统 | Ubuntu 22.04 |
| ROS2 | Humble |
| Python | 3.10+ |
| 构建工具 | `colcon`, `cmake`, `g++` |

### 2.1 系统依赖

```bash
sudo apt update && sudo apt install -y \
  build-essential cmake pkg-config \
  libeigen3-dev libyaml-cpp-dev libevdev-dev \
  ros-humble-pinocchio ros-humble-desktop \
  patchelf
```

### 2.2 OSQP 求解器（robot_controller 运行时依赖）

```bash
# OSQP
cd ~ && git clone --branch release-0.6.3 --recursive https://github.com/osqp/osqp.git
cd osqp && mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=~/osqp/install ..
make -j$(nproc) && make install

# OsqpEigen
cd ~ && git clone https://github.com/robotology/osqp-eigen.git
cd osqp-eigen && mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=~/osqp-eigen/install -DCMAKE_PREFIX_PATH=~/osqp/install ..
make -j$(nproc) && make install
```

编译前设置环境变量：

```bash
export CMAKE_PREFIX_PATH=~/osqp-eigen/install:~/osqp/install:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=~/osqp/install/lib:~/osqp-eigen/install/lib:$LD_LIBRARY_PATH
```

### 2.3 厂商 SDK（按需）

`teleop_adapter` 根据目标机械臂按需链接厂商 SDK：

| adapter_type | 厂商 SDK | 说明 |
|--------------|----------|------|
| `teleop` | 无 | Nexus 主臂串口通信 |
| `ar5` / `ar5_suction_cup` / `ar5_gripper` | Rokae libxCoreSDK | 已内置在 `external/rokae_ar5/` |
| `y1` | Y1 SDK | 需放置到 `external/imeta_y1/lib/${ARCH}/` |
| `franka_hand` | libfranka | 客户自行安装，见 §5 |

---

## 3. 部署与编译

### 3.1 创建工作空间

**方式 A：解压 tar 包**

```bash
mkdir -p ~/nexus-ws/src
tar xzf nexus-sdk-release-*.tar.gz -C ~/nexus-ws/src
cd ~/nexus-ws
```

**方式 B：克隆发布分支**

```bash
git clone --branch jd_release <repo_url> ~/nexus-ws
cd ~/nexus-ws
# 发布分支中 ROS 包位于 src/ 子目录
```

### 3.2 编译

```bash
source /opt/ros/humble/setup.bash

colcon build \
  --packages-up-to teleop_adapter robot_controller nexus_manage human_data \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

编译过程说明：

- `infra_msg`：从 `.msg`/`.srv` 生成 C++/Python 接口
- `robot_controller` / `nexus_manage` / `human_data`：仅执行 `install()`，拷贝预编译 `.so` 和可执行文件到 `install/`
- `teleop_adapter`：**完整编译**所有 `.cpp`，链接厂商 SDK，生成 `teleop_arm_driver_main`

### 3.3 验证安装

```bash
# 检查预编译节点
ros2 pkg executables robot_controller    # arm_control_node
ros2 pkg executables nexus_manage        # teleop_manager_node
ros2 pkg executables human_data          # human_data_solver_node

# 检查 teleop_adapter（客户编译产物）
ros2 pkg executables teleop_adapter      # teleop_arm_driver_main
ldd install/teleop_adapter/lib/teleop_adapter/teleop_arm_driver_main
```

---

## 4. 系统架构与数据流

以 **Nexus-Arm V15 Right → Franka FR3v2.1 + Hand** 主从遥操作为例：

```
Master 侧                              Slave 侧
─────────                              ─────────
teleop_adapter (teleop)                teleop_adapter (franka_hand) ← 客户实现
    │ joint_states                         │ joint_cmd
    ▼                                      ▼
human_data                             robot_controller (预编译)
    │ human_data                           │ end_effector_cmd
    ▼                                      ▼
nexus_manage (预编译)  ◀──── DDS ────▶  nexus_manage (预编译)
    │ 状态机 / 映射                          │ 状态机 / 映射
    ▼                                      ▼
robot_controller (预编译)              human_data (预编译)
```

- **预编译包**：客户不可修改源码，通过 YAML 配置和 Launch 文件调参
- **teleop_adapter**：客户可新增 adapter 类，对接任意机械臂 SDK

---

## 5. 适配其他品牌机械臂（teleop_adapter）

### 5.1 适配流程概览

1. **继承 `DeviceAdapter` 基类**，实现硬件通信接口
2. **在 `createAdapter()` 中注册**新的 `adapter_type`
3. **修改 `CMakeLists.txt`** 链接厂商 SDK
4. **编写 YAML 配置**和 Launch 文件
5. **本地编译** `teleop_adapter` 并验收

### 5.2 参考实现

| 机械臂 | adapter_type | 参考文件 |
|--------|--------------|----------|
| Nexus 主臂 | `teleop` | `src/adapters/teleop_arm_adapter.cpp` |
| Rokae AR5 | `ar5` | `src/adapters/ar5_arm_adapter.cpp` |
| Rokae AR5 + 吸盘 | `ar5_suction_cup` | `src/adapters/ar5_suction_cup_adapter.cpp` |
| Rokae AR5 + 夹爪 | `ar5_gripper` | `src/adapters/ar5_gripper_adapter.cpp` |
| Y1 | `y1` | `src/adapters/y1_arm_adapter.cpp` |

### 5.3 必须实现的 DeviceAdapter 接口

```cpp
class YourArmAdapter : public DeviceAdapter {
public:
    // 设备握手与初始化
    bool deviceHandshake() override;
    bool configureDevice() override;

    // 电机使能/失能
    bool enableMotors() override;
    bool disableMotors() override;

    // 控制与反馈
    bool sendMotorCommands(const std::vector<MotorCommand>& commands) override;
    bool readMotorStates(std::vector<MotorState>& states) override;

    // 可选：六维力反馈
    bool readExternalWrench(std::array<double, 6>& wrench) override;

    // 后台线程
    void readThreadFunc() override;
    void writeThreadFunc() override;
};
```

### 5.4 注册新 adapter

在 `src/nodes/teleop_arm_driver_node.cpp` 的 `createAdapter()` 中添加分支：

```cpp
#include "teleop_adapter/adapters/your_arm_adapter.hpp"

// ...
} else if (adapter_type_ == "your_robot") {
    adapter = std::make_unique<YourArmAdapter>(
        device_config, config.port, config.init_joint_positions);
```

### 5.5 编写配置文件

复制现有配置为模板，例如 `config/slave_fr3v2_1_hand_single.yaml`：

```yaml
teleop_arm_driver_node:
  ros__parameters:
    adapter_type: "your_robot"       # 与 createAdapter() 中一致
    num_of_dofs: 7
    right_arm_port: "192.168.1.100"  # 机器人 IP 或串口
    right_arm_num_of_dofs: 7
    right_arm_feedback_rate: 500.0
    joint_names: ["joint1", "joint2", ...]
    default_dof_pos: [0.0, -0.785, ...]
```

### 5.6 Franka FR3v2.1 + Hand 适配

Franka 从臂的完整实现指南（含 libfranka 集成、线程架构、夹爪单位约定、分阶段验收）见：

**[`docs/franka_teleop_adapter_real_deployment.md`](franka_teleop_adapter_real_deployment.md)**

（若发布包内未包含该文档，请向供应商索取或参考 `teleop_adapter/docs/` 目录。）

### 5.7 重新编译 teleop_adapter

修改 adapter 源码后，只需重新编译该包：

```bash
cd ~/nexus-ws
colcon build --packages-select teleop_adapter --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

---

## 6. 启动系统

### 6.1 仿真（无需 teleop_adapter 从臂）

```bash
ros2 launch nexus_manage nexus_arm_v15_right_to_fr3v2_1_hand_sim_system.launch.py
```

### 6.2 实机全系统

**Master 侧**（Nexus 主臂）：

```bash
ros2 launch nexus_manage nexus_arm_v15_right_to_fr3v2_1_hand_real_system.launch.py \
  role:=master
```

**Slave 侧**（Franka 从臂，adapter 实现完成后）：

```bash
ros2 launch nexus_manage nexus_arm_v15_right_to_fr3v2_1_hand_real_system.launch.py \
  role:=slave
```

### 6.3 单节点调试

```bash
# 仅启动从臂 teleop（验证 adapter）
ros2 launch teleop_adapter slave_fr3v2_1_hand_single.launch.py

# 仅启动从臂 controller
ros2 launch robot_controller slave_single_fr3v2_1_hand.launch.py
```

---

## 7. 调参文档

| 文档 | 内容 |
|------|------|
| [`docs/franka_tuning_guide.md`](franka_tuning_guide.md) | 从臂 controller / 夹爪 / 力反馈调参 |
| [`docs/franka_teleop_adapter_real_deployment.md`](franka_teleop_adapter_real_deployment.md) | Franka adapter 实现与实机部署 |
| `teleop_adapter/docs/component.md` | teleop_arm_driver_node 架构说明 |

> 预编译包的 YAML 配置文件位于各包的 `config/` 和 `launch/` 目录，可直接修改参数而无需重新编译（`robot_controller`、`nexus_manage`、`human_data`）。

---

## 8. 常见问题

### Q: colcon build 报找不到预编译库

确认当前 CPU 架构与发布包中 `lib/` 目录匹配：

```bash
uname -m                          # x86_64 或 aarch64
ls src/robot_controller/lib/      # 应包含对应架构子目录
```

若缺少目标架构的库，请联系供应商获取对应平台的发布包。

### Q: `Unsupported adapter_type: franka_hand`

表示 `teleop_adapter` 中尚未实现或未编译 `franka_hand` adapter。请按 §5.6 完成实现后重新编译 `teleop_adapter`。

### Q: 修改 robot_controller 参数后需要重新编译吗？

不需要。YAML 配置在运行时加载，修改 `config/*.yaml` 后重启对应 launch 即可。

### Q: 如何确认使用的是预编译库而非本地编译？

预编译包的 `CMakeLists.txt` 仅含 `install()` 指令，**不含** `add_library()` / `add_executable()` 和 `src/` 目录。`teleop_adapter` 则包含完整 `src/` 和原始 `CMakeLists.txt`。

### Q: patchelf / RPATH 相关

发布包制造时已对预编译可执行文件设置 `$ORIGIN/..` 相对 RPATH。若仍报找不到 `.so`，检查 `lib/${ARCH}/` 下是否包含所需的外部依赖库。

---

## 9. 技术支持

- 预编译包版本问题、缺少架构：联系供应商
- `teleop_adapter` adapter 开发：参考 §5 及 `teleop_adapter/docs/`
- 系统调参：参考 §7 调参文档

---

*文档版本：2026-06 — Nexus SDK 混合发布包（预编译库 + teleop_adapter 源码）*
