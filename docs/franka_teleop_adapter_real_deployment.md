# Franka FR3v2.1 + Hand：teleop_adapter 实机适配与部署

> **主臂**：Nexus-Arm V15 Right  
> **从臂**：Franka Research 3 v2.1 + Franka Hand  
> **本文范围**：在 `teleop_adapter` 中实现 `franka_hand` adapter，并完成从臂实机部署与验收。

仿真阶段由 `mujoco_sim` 订阅 `/fr3v2_1/robot/joint_cmd`，**不经过** `teleop_adapter`。实机从臂侧必须实现本文所述 adapter，否则 `teleop_arm_driver_main` 启动即报 `Unsupported adapter_type: franka_hand`。

从臂 controller / 夹爪调参见 [franka_tuning_guide.md](./franka_tuning_guide.md)。

---

## 目录

1. [adapter 在系统中的位置](#1-adapter-在系统中的位置)
2. [前置条件](#2-前置条件)
3. [仓库已有骨架](#3-仓库已有骨架)
4. [实现 FrankaArmAdapter](#4-实现-frankaarmadapter)
5. [注册 adapter 与编译](#5-注册-adapter-与编译)
6. [yaml 与实机配置](#6-yaml-与实机配置)
7. [分阶段验收](#7-分阶段验收)
8. [全系统实机启动](#8-全系统实机启动)
9. [夹爪与单位约定](#9-夹爪与单位约定)
10. [故障排查](#10-故障排查)

---

## 1. adapter 在系统中的位置

### 1.1 Slave 侧数据流

```
robot_controller (arm_control_node)
    │ 订阅 /robot/end_effector_cmd（来自 Master nexus_manage）
    │ 发布 /fr3v2_1/robot/joint_cmd   (infra_msg/JointMitControl, 8 DOF)
    ▼
teleop_adapter (teleop_arm_driver_main)   ← 本文实现
    │ libfranka FCI，典型 1000 Hz
    ▼
Franka FR3v2.1 + Hand
    │ joint_states / external_wrench
    ▼
human_data (human_data_solver_node)
    │ /fr3v2_1/robot/human_data
    ▼
（经 DDS 回传 Master nexus_manage）
```

### 1.2 与 AR5 adapter 的本质区别

| 项目 | AR5 (`ar5_suction_cup`) | Franka (`franka_hand`) |
|------|-------------------------|------------------------|
| SDK | Rokae libxCoreSDK | libfranka / FCI |
| 连接参数 | `right_arm_port` = 控制器 IP | `right_arm_port` = FCI 机器人 IP |
| 控制频率 | 500 Hz | 1000 Hz |
| 臂关节 0–6 | MIT 力矩（SDK 回调） | MIT 力矩（`robot.control()` 回调） |
| 关节 7 | HTTP 吸盘角度 (rad) | `finger_joint1` 开合 (m)，0～0.04 |
| 参考实现 | `ar5_suction_cup_adapter.cpp` | 业务逻辑不可复用（SDK 不同），但可参考 `ar5_arm_adapter.cpp` 三线程架构与接口规范 |

---

## 2. 前置条件

### 2.1 硬件

- Franka FR3v2.1 + Franka Hand，FCI 已激活
- 工控机与控制柜网线直连（或同网段低延迟链路）
- 已完成 Franka Desk 基础安全配置（碰撞阈值、工作空间等）

### 2.2 软件

| 组件 | 要求 |
|------|------|
| OS | Ubuntu 22.04 |
| ROS2 | Humble |
| libfranka | 版本与机器人固件匹配（[官方兼容性表](https://frankaemika.github.io/docs/libfranka.html)） |
| nexus-sdk | 完整源码（adapter 需在本地编译进 `teleop_adapter`） |

### 2.3 建议顺序

1. 先跑通仿真：`ros2 launch nexus_manage nexus_arm_v15_right_to_fr3v2_1_hand_sim_system.launch.py`
2. 再按本文实现 adapter 并联调 Slave 单节点
3. 最后双机 / 跨子网全系统验收

---

## 3. 仓库已有骨架

以下文件**已存在**，adapter 实现完成后可直接使用：

| 文件 | 说明 |
|------|------|
| `teleop_adapter/config/slave_fr3v2_1_hand_single.yaml` | `adapter_type: franka_hand`，FCI IP、关节名 |
| `teleop_adapter/launch/slave_fr3v2_1_hand_single.launch.py` | 从臂 teleop 单机 launch |
| `robot_controller/config/slave_single_fr3v2_1_hand_config.yaml` | 从臂 URDF、关节名、`robot_type: franka_hand` |
| `human_data/config/fr3v2_1_hand_human_data_config.yaml` | 从臂 FK |
| `nexus_manage/config/nexus-arm_v15_right_to_fr3v2_1_hand_manage.yaml` | 主从映射、夹爪缩放 |
| `nexus_manage/launch/nexus_arm_v15_right_to_fr3v2_1_hand_real_system.launch.py` | 实机全系统 launch |

**需要新增 / 实现**：

| 文件 | 说明 |
|------|------|
| `include/teleop_adapter/adapters/franka_arm_adapter.hpp` | 类声明 |
| `src/adapters/franka_arm_adapter.cpp` | libfranka 实现 |
| `teleop_arm_driver_node.cpp` | `createAdapter()` 增加 `franka_hand` 分支 |
| `CMakeLists.txt` | 链接 libfranka |

---

## 4. 实现 FrankaArmAdapter

### 4.1 类接口（建议与 AR5 对齐）

```cpp
// franka_arm_adapter.hpp
static constexpr int FRANKA_ARM_DOF = 7;
static constexpr int FRANKA_HAND_DOF = 1;
static constexpr int FRANKA_TOTAL_DOF = 8;

class FrankaArmAdapter : public DeviceAdapter {
public:
    FrankaArmAdapter(const DeviceAdapterConfig& config,
                     const std::string& robot_ip,
                     const std::string& local_ip,
                     const std::vector<double>& init_joint_positions);
    // 实现 DeviceAdapter 纯虚函数：
    // deviceHandshake, configureDevice, enableMotors, disableMotors,
    // sendMotorCommands, readMotorStates, readExternalWrench,
    // readThreadFunc, writeThreadFunc
};
```

### 4.2 推荐线程架构

参照 `ar5_arm_adapter.cpp`：

```
线程 1 — libfranka control() 回调（~1 ms）
  ├─ 读 state.q / state.dq
  ├─ 读 cmd_buffer（上层 MIT 命令）
  ├─ 计算 tau_desired = kp*(q_des-q) + kd*(dq_des-dq) + tau_ff
  └─ 返回 franka::Torques

线程 2 — readThreadFunc（yaml feedback_rate，建议 500～1000 Hz）
  └─ 聚合 7 臂 + 1 夹爪状态 → latest_states

线程 3 — writeThreadFunc
  └─ 检测 FCI 异常、保护停，更新 DeviceStatus

线程 4（可选）— gripperControlLoop
  └─ 监听 cmd_buffer[7].position (m)，调用 franka::Gripper::move()
```

夹爪**不要**在 1 kHz 臂回调里做阻塞 HTTP/SDK 调用；独立线程异步处理。

### 4.3 deviceHandshake()

典型步骤：

1. `franka::Robot robot(robot_ip)` 连接
2. `robot.automaticErrorRecovery()` 清除可恢复错误
3. 设置碰撞行为（与现场安全策略一致）
4. `franka::Gripper gripper(robot_ip)` 初始化夹爪（若使用独立 Gripper 类）
5. Move 到 `init_joint_positions`：
   - 索引 0–6：臂关节 (rad)
   - 索引 7：夹爪目标宽度 (m)，复位建议 `0.04`（全开，与 manage yaml 对齐）

### 4.4 configureDevice()

启动 `robot.control()` 力矩模式：

```cpp
robot_->control([this](const franka::RobotState& state,
                       franka::Duration) -> franka::Torques {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    franka::Torques out{};
    for (int i = 0; i < 7; ++i) {
        const auto& c = cmd_buffer_[i];
        out.tau[i] = c.kp * (c.position - state.q[i])
                   + c.kd * (c.velocity - state.dq[i])
                   + c.torque;
    }
    return out;
});
```

### 4.5 sendMotorCommands()

| 索引 | 字段含义 | 单位 | adapter 行为 |
|------|----------|------|--------------|
| 0–6 | MIT position/velocity/torque/kp/kd | rad, rad/s, N·m | 写入 `cmd_buffer_`，由 control 回调消费 |
| 7 | `position` | **m** | 写入夹爪目标；`gripperControlLoop` 调用 `Gripper::move(width)` |

`robot_controller` 对 `robot_type: franka_hand` 已将夹爪命令换算为米制 prismatic 目标，adapter **直接执行**即可。

### 4.6 readMotorStates()

返回 8 维 `MotorState`：

| 索引 | position 单位 | 来源 |
|------|---------------|------|
| 0–6 | rad | `state.q[i]` |
| 7 | m | `Gripper::readOnce()` 或 Hand 关节反馈 |

### 4.7 readExternalWrench()

从 `RobotState::O_F_ext_hat_K` 填入 6D `[Fx,Fy,Fz,Tx,Ty,Tz]`，供主臂力反馈可选通路。

节点会发布到 `/fr3v2_1/robot/external_wrench`。

---

## 5. 注册 adapter 与编译

### 5.1 createAdapter() 注册

在 `src/teleop_adapter/src/nodes/teleop_arm_driver_node.cpp` 的 `createAdapter()` 中增加：

```cpp
#include "teleop_adapter/adapters/franka_arm_adapter.hpp"

// ...
} else if (adapter_type_ == "franka_hand") {
    std::string robot_ip = config.port;
    std::string local_ip = config.local_ip;
    if (robot_ip.empty() || local_ip.empty()) {
        RCLCPP_ERROR(owner_->get_logger(),
            "franka_hand requires right_arm_port (FCI IP) and right_arm_local_ip");
        return nullptr;
    }
    adapter = std::make_unique<FrankaArmAdapter>(
        device_config, robot_ip, local_ip, config.init_joint_positions);
```

### 5.2 CMakeLists.txt

```cmake
option(BUILD_FRANKA_ADAPTER "Build Franka arm adapter (requires libfranka)" ON)
if(BUILD_FRANKA_ADAPTER)
  find_package(Franka REQUIRED)
  add_library(franka_arm_adapter src/adapters/franka_arm_adapter.cpp)
  target_include_directories(franka_arm_adapter PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_link_libraries(franka_arm_adapter device_adapter Franka::Franka)
endif()
```

将 `franka_arm_adapter` 加入 `teleop_arm_driver_main` 的 `target_link_libraries`，并 `install(TARGETS franka_arm_adapter ...)`。

编译：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select infra_msg
source install/setup.bash
colcon build --symlink-install --packages-select teleop_adapter
source install/setup.bash
```

验证链接：

```bash
ldd install/teleop_adapter/lib/teleop_adapter/teleop_arm_driver_main | grep franka
```

---

## 6. yaml 与实机配置

### 6.1 teleop_adapter — slave_fr3v2_1_hand_single.yaml

部署前修改 FCI 网络：

```yaml
slave_arm_adapter:
  ros__parameters:
    robot_name: "fr3v2_1"
    adapter_type: "franka_hand"
    right_arm_port: "172.16.0.2"       # FCI 机器人 IP
    right_arm_local_ip: "172.16.0.1"   # 工控机网卡 IP
    right_arm_feedback_rate: 1000
    right_arm_num_of_dofs: 8
    right_arm_init_joint_positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.04]
    num_of_dofs: 8
    joint_names:
      - fr3v2_1_joint1 … fr3v2_1_joint7
      - fr3v2_1_finger_joint1
    joint_mapping: [0, 1, 2, 3, 4, 5, 6, 7]
```

`joint_names` 必须与 `slave_single_fr3v2_1_hand_config.yaml`、`fr3v2_1_hand_human_data_config.yaml` **完全一致**。

### 6.2 robot_controller — 实机模式

`slave_single_fr3v2_1_hand_config.yaml`：

```yaml
is_simulation: false   # 实机必须 false
```

`arm_control_fr3v2_1_hand.yaml` 当前由 AR5 模板派生，实机前需按 FR3 重新标定关节限位与阻抗增益。

### 6.3 跨子网（可选）

1. 编辑 `nexus_manage/launch/nexus_arm_v15_right_to_fr3v2_1_hand_real_system_cross_subnet.launch.py` 顶部 `_MASTER_*` / `_SLAVE_*` 网络常量
2. 编辑 `nexus_manage/config/slave_registry.yaml`：

```yaml
slaves:
  - id: fr3v2_1_01
    ip: 192.168.8.78
    local_ips: ["192.168.8.78"]
    network_interface: eno1
```

Slave 启动时：`robot_id:=fr3v2_1_01`（覆盖 `robot_name` 前缀）。

---

## 7. 分阶段验收

**按顺序执行，不要跳过单节点验证。**

### 阶段 1：仅 teleop_adapter

```bash
ros2 launch teleop_adapter slave_fr3v2_1_hand_single.launch.py
ros2 topic hz /fr3v2_1/robot/joint_states
ros2 topic echo /fr3v2_1/robot/arm_status --once
```

期望：8 关节连续反馈；`arm_status` 显示已连接、已使能；日志无 FCI 异常。

### 阶段 2：手动 joint_cmd 探针

```bash
# 仅测试夹爪：finger_joint1 = 0.02 m
ros2 topic pub /fr3v2_1/robot/joint_cmd infra_msg/msg/JointMitControl "{
  joint_position: [0,0,0,0,0,0,0, 0.02],
  joint_velocity: [0,0,0,0,0,0,0, 0],
  torque:         [0,0,0,0,0,0,0, 0],
  kp:             [0,0,0,0,0,0,0, 100],
  kd:             [0,0,0,0,0,0,0, 1]
}" --once
```

期望：夹爪动作；`joint_states` 第 8 维跟随变化。

### 阶段 3：Slave 三节点（不接 Master）

```bash
ros2 launch teleop_adapter slave_fr3v2_1_hand_single.launch.py
ros2 launch human_data fr3v2_1_hand_human_data_solver.launch.py
ros2 launch robot_controller slave_single_fr3v2_1_hand.launch.py
```

期望：三节点无报错；`human_data` 有 `/fr3v2_1/robot/human_data` 输出。

### 阶段 4：全系统

见下一节。

---

## 8. 全系统实机启动

### 8.1 双机同网

**Slave 工控机（先启动）**

```bash
source /opt/ros/humble/setup.bash
source ~/nexus-sdk/install/setup.bash
export ROS_DOMAIN_ID=18

ros2 launch nexus_manage nexus_arm_v15_right_to_fr3v2_1_hand_real_system.launch.py role:=slave
```

**Master 机（Nexus V15，后启动）**

```bash
ros2 launch nexus_manage nexus_arm_v15_right_to_fr3v2_1_hand_real_system.launch.py role:=master
```

### 8.2 单机首检

```bash
ros2 launch nexus_manage nexus_arm_v15_right_to_fr3v2_1_hand_real_system.launch.py
```

### 8.3 跨子网

```bash
# Slave
ros2 launch nexus_manage nexus_arm_v15_right_to_fr3v2_1_hand_real_system_cross_subnet.launch.py \
  role:=slave robot_id:=fr3v2_1_01

# Master
ros2 launch nexus_manage nexus_arm_v15_right_to_fr3v2_1_hand_real_system_cross_subnet.launch.py \
  role:=master
```

### 8.4 遥操作步骤

1. 等待各节点自检完成
2. 小键盘 `1`：主从复位
3. `R` + `Q`：进入遥操作
4. 松开 `R`：位置保持

### 8.5 关键 Topic

```bash
ros2 topic list | grep -E 'fr3v2_1|teleop|robot'
```

| Topic | 方向 | 说明 |
|-------|------|------|
| `/fr3v2_1/robot/joint_cmd` | controller → teleop | 8 DOF MIT |
| `/fr3v2_1/robot/joint_states` | teleop → human_data | 关节反馈 |
| `/fr3v2_1/robot/external_wrench` | teleop → 力反馈 | 可选 |
| `/teleop/human_data` | 主臂 FK | Master 侧 |
| `/robot/end_effector_cmd` | manage → controller | 主从映射结果 |

---

## 9. 夹爪与单位约定

整条链路（Y1 范式，米制末端）：

```
主臂 human_data:  gripper = gripper_Joint + 0.628        (V15 Right)
nexus_manage:     robot.gripper = teleop.gripper × 0.063694
robot_controller: opening_mm → finger_joint1 目标 (m)
teleop_adapter:   执行 finger_joint1，范围 0（闭合）～ 0.04（全开）
```

**复位对齐**

- 主臂全开：`gripper_Joint = 0` rad
- 从臂全开：`finger_joint1 = 0.04` m（`nexus-arm_v15_right_to_fr3v2_1_hand_manage.yaml` 中 `robot_pos_values: [0.04]`）

**重算缩放因子**

`gripper_scaling_factor` 由主从夹爪行程比计算：

```
gripper_scaling_factor = slave_max_opening_distance / master_max_opening_distance
```

例如 Franka Hand：`40.0 / 20.58 ≈ 1.943`（以 mm 为单位计算，结果需换算到目标量纲）。

> 若需自动化计算，可参照此公式编写本地脚本。

结果写入 `nexus-arm_v15_right_to_fr3v2_1_hand_manage.yaml` 的 `gripper_scaling_factor`；从臂行程 `gripper_config_fr3v2_1_hand.yaml` 中 `slave_max_opening_distance: 40.0`（mm）。

---

## 10. 故障排查

| 现象 | 原因 | 处理 |
|------|------|------|
| `Unsupported adapter_type: franka_hand` | adapter 未实现或未编译 | 完成 §4–§5 后重建 `teleop_adapter` |
| FCI 连接失败 | IP / 网线 / 防火墙 | 检查 `right_arm_port`、`right_arm_local_ip`，ping 控制柜 |
| Franka 保护停 | 碰撞、超速、奇异位形 | Desk 或 `automaticErrorRecovery()`，检查初始位姿 |
| `joint_cmd` 有订阅但臂不动 | 未进力矩模式或 kp 全零 | 查 adapter 日志；确认 controller 已下发非零 kp |
| 臂动、夹爪不动 | 关节 7 未接入 Gripper API | 查 `sendMotorCommands[7]` 与 gripper 线程 |
| 主从不跟手 | DDS 不通 | 统一 `ROS_DOMAIN_ID`；检查 CycloneDDS 网卡配置 |
| 夹爪行程偏差 | 缩放未标定 | §9 重算 `gripper_scaling_factor` 并现场微调 |
| 抖动 / 撞限位 | 控制参数未标定 | 按 FR3 修改 `arm_control_fr3v2_1_hand.yaml` |

---

## 附录：实现工作清单

| 优先级 | 任务 | 验收标准 |
|--------|------|----------|
| P0 | 安装匹配版本 libfranka | `franka::Robot` 连接成功 |
| P0 | 实现 `FrankaArmAdapter` 全套接口 | 阶段 1–3 通过 |
| P0 | 7 轴 MIT 跟随 `joint_cmd` | 臂平滑运动，无突变 |
| P0 | `finger_joint1` 0～0.04 m | 主臂夹爪键控制开合 |
| P1 | `external_wrench` 发布 | 主臂力反馈可选 |
| P0 | `arm_control_fr3v2_1_hand.yaml` 实机标定 | 无限位撞墙 |
| P0 | 双机全系统 launch | 遥操作跟随正常 |
| P0 | 急停 / 保护停恢复流程 | 无非预期运动 |

---

*文档版本：2026-06 — Franka `franka_hand` teleop_adapter 实机适配与部署。*
