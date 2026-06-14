# Franka 从臂调参指南（V15 Right → FR3v2.1 + Hand）

> 面向 **仿真联调** 与 **实机标定**。  
> 实机 `teleop_adapter` 部署见 [franka_teleop_adapter_real_deployment.md](./franka_teleop_adapter_real_deployment.md)。

本文以从臂两个核心 yaml 为主线：

| 优先级 | 文件 | 作用 |
|--------|------|------|
| **P0** | `robot_controller/config/arm_control_fr3v2_1_hand.yaml` | 7 轴 IK、阻抗 MIT、安全域、外力估计 |
| **P0** | `robot_controller/config/gripper_config_fr3v2_1_hand.yaml` | 第 8 轴 `finger_joint1`（m）MIT 与力反馈 |
| P1 | `nexus_manage/config/nexus-arm_v15_right_to_fr3v2_1_hand_manage.yaml` | 主从复位、末端/夹爪缩放 |
| P2 | `robot_controller/config/master_arm_control_nexus_v15_right.yaml` | 主臂力反馈、摩擦补偿 |
| P2 | `robot_controller/config/gripper_config_nexus_v15_right.yaml` | 主臂夹爪行程与力反馈接收 |

修改 yaml 后：

```bash
colcon build --symlink-install --packages-select robot_controller nexus_manage
source install/setup.bash
```

---

## 目录

1. [调参顺序](#1-调参顺序)
2. [arm_control_fr3v2_1_hand.yaml](#2-arm_control_fr3v2_1_handyaml)
3. [gripper_config_fr3v2_1_hand.yaml](#3-gripper_config_fr3v2_1_handyaml)
4. [nexus_manage 主从映射](#4-nexus_manage-主从映射)
5. [主臂与其它配置](#5-主臂与其它配置)
6. [现象 → 参数对照](#6-现象--参数对照)

---

## 1. 调参顺序

建议按以下顺序，**每步只改一类参数**，便于归因：

1. **复位对齐**（`nexus-arm_v15_right_to_fr3v2_1_hand_manage.yaml`）  
   小键盘 `1` 后主从腕部姿态、夹爪全开一致（从臂 `finger_joint1 = 0.04 m`）。

2. **末端跟踪**（`vqp_clik_gain`、`motion_planning_kp/kd`）  
   消除跟手滞后或 MIT 抖动。

3. **奇异 / 零空间漂移**（`vqp_ref_joint_*`、`vqp_arm_angle_*`、`vqp_max_total_joint_change`）  
   长时遥操作肘部姿态飘、关节蹭限位时调。

4. **夹爪行程与刚度**（`gripper_scaling_factor` → `gripper_config_fr3v2_1_hand.yaml`）  
   主从开合比例、闭合速度、仿真抖动。

5. **力反馈**（可选，实机稳定后）  
   从臂 `force_feedback_use_fsm`、主臂 `ee_ff_gain`。

6. **安全与 CBF**（可选）  
   `redundant_ik_safety` 工作空间盒；生成 `cbf_fr3v2_1_spherized.urdf` 后开 `enable_cbf`。

---

## 2. arm_control_fr3v2_1_hand.yaml

从臂 `arm_control_node` 加载，参数块位于 `master_arm_controller` 命名空间下（该命名空间在代码中用于区分主从臂控制器参数块）。

> **注意**：实机前须按 FR3 关节限位与力矩上限重标定 §2.8 增益。

### 2.1 基础设置（§1）

| 参数 | 默认 | 说明与建议 |
|------|------|------------|
| `is_simulation` | `true` | 仿真 `true`（不加电机惯量补偿）；实机 `false` |
| `control_publish_rate` | `500` | MIT 下发频率 (Hz)，与从臂控制环一致；Franka FCI 1000 Hz 由 `teleop_adapter` 处理 |
| `state_publish_rate` | `10` | 节点状态发布，一般不改 |
| `startup_check_timeout` | `10.0` | 自检超时 (s)，节点未就绪时可略增 |

### 2.2 电机等效参数（§2）

| 参数 | 说明与建议 |
|------|------------|
| `motor_params.rotor_inertia` | 各轴转子惯量；仿真须与 MuJoCo `joint armature` 对齐 |
| `motor_params.gear_ratio` | 减速比；当前为 AR5 模板值，**实机按 FR3 或仿真 MJCF 重填** |

`I_equiv = rotor_inertia × gear_ratio²`，影响动力学补偿与 `use_motor_inertia`。

### 2.3 关节限制（§3）

| 参数 | 说明与建议 |
|------|------------|
| `joint_limits.max_velocity` | 轨迹规划速度上限 (rad/s)；遥操作突变大时可整体 ↓ |
| `joint_limits.max_acceleration` | 加速度上限；抖动明显时可 ↓ |
| `joint_limits.max_jerk` | 加加速度上限；平滑性不足时可 ↓ |

与 IK 内 `vqp_joint_velocity_limits` 配合：前者偏规划，后者偏 QP 硬约束。

### 2.4 逆运动学（§4，`ik_algorithm: velocity_level_qp`）

当前默认 **速度级 QP + CLIK**，下列 `vqp_*` 生效；`qp_*` / DLS 段仅在切换 `ik_algorithm` 后才有意义。

#### 算法与共用项

| 参数 | 默认 | 调参建议 |
|------|------|----------|
| `ik_algorithm` | `velocity_level_qp` | 7 轴冗余遥操作推荐保持；极端奇异可试 `qp_wls_task_slack` |
| `weight_ik` | `[1,1,1, 0.3,0.3,0.3]` | 末端误差权重 [x,y,z,r,p,y]；位置跟不上可 ↑ 平移；姿态抖可 ↓ 旋转 |
| `epsilon` | `0.02` | IK 收敛阈值；↑ 更早停止、略钝；↓ 更贴目标、算力↑ |
| `ik_iter_max` | `500` | 单周期迭代上限；零空间漂移大时可 ↓ |
| `ik_dt` | `0.07` | 内层步长；建议 0.05～0.09 |
| `ik_hold_skip_solver_when_within_tolerance` | `false` | `true` 可在末端已到位时跳过 IK，减仿真“坠感”与冗余蠕动 |

#### 速度级 QP 核心（`vqp_*`）

| 参数 | 默认 | 调参建议 |
|------|------|----------|
| `vqp_clik_gain` | `[6,6,6, 5,5,5]` | **跟手性第一旋钮**。滞后 ↑ 平移/旋转增益；抖动 ↓ 增益或 ↑ `vqp_regularization` |
| `vqp_slack_penalty` | `6000` | 任务松弛惩罚 ρ；↑ 更钉死末端；奇异/不可达时 ↓ 略增宽容 |
| `vqp_regularization` | `0.00025` | 基础 ‖q̇‖² 正则；关节速度噪或抖时可 ↑ |
| `vqp_adaptive_reg_enable` | `true` | 接近奇异时自动增阻尼，一般保持开启 |
| `vqp_adaptive_reg_sigma_threshold` | `0.015` | 自适应阻尼触发奇异值阈值 |
| `vqp_adaptive_reg_max_boost` | `0.008` | 奇异时额外正则峰值；奇异区抖动 ↑ 此项 |
| `vqp_joint_velocity_limits` | 全 `1.0` rad/s | 按 FR3 手册与现场速度收紧；抑制甩动 |
| `vqp_position_limit_gain` | `10.0` | 接近关节限位时自动缩小允许速度；蹭限位 ↑ |
| `vqp_max_qdot_norm` | `2.0` | 单周期 ‖q̇‖ 上限；突变大时可 ↓ |
| `vqp_max_total_joint_change` | `0.1` rad | 单周期关节变化硬上限；**抖动优先 ↓**（如 0.05）；跟手迟滞可 ↑ |

#### 零空间与肘部姿态

| 参数 | 默认 | 调参建议 |
|------|------|----------|
| `vqp_ref_joint_enable` | `true` | 防冗余 DOF 漂向奇异；长时静止漂移 ↑ `vqp_ref_joint_weight` |
| `vqp_ref_joint_weight` | `0.003` | 零空间回参考关节强度；过大末端略钝 |
| `vqp_ref_joint_policy` | `last_q_target` | 连续性好；固定臂形可改 `user_specified` |
| `vqp_arm_angle_enable` | `true` | 肩-肘-腕平面（swivel）约束 |
| `vqp_arm_angle_weight` | `0.03` | 肘部姿态漂移大时 ↑ |
| `vqp_arm_angle_ref_joint_positions` | 见 yaml | `user_specified` 时的参考关节角，按现场 home 调整 |
| `vqp_arm_angle_min/max` | `±0.3` rad | 相对参考平面的臂角允许范围 |
| `vqp_manipulability_grad_enable` | `true` | 软引导远离奇异；静止漂移异常时可关 |

#### 其它 IK 段（默认未启用）

| 参数 | 说明 |
|------|------|
| `vqp_cbf_enable` | 速度级可操作度 CBF 硬约束，默认 `false` |
| `vqp_barrier_enable` | Barrier 函数，默认 `false` |
| `vqp_ee_velocity_limits` | 末端线/角速度盒，注释掉=不限 |

### 2.5 IK 初值与安全域（§5）

| 参数 | 默认 | 调参建议 |
|------|------|----------|
| `redundant_numeric_ik_init` | `last_ik_output` | warm start，遥操作保持默认 |
| `redundant_ik_safety.enable_task_space_box` | `true` | 任务空间软盒，越界前裁剪 IK 目标 |
| `redundant_ik_safety.task_space.limits` | x/y/z 盒 | 按 Franka 工作空间缩小，防不可达 |
| `redundant_ik_safety.joint_limit` | 见 yaml | 须与 URDF `fr3v2_1_joint*` 一致；`safety_margin` 越大越早软限 |

### 2.6 CBF（§6）

| 参数 | 默认 | 调参建议 |
|------|------|----------|
| `enable_cbf` | `false` | 需 `cbf_fr3v2_1_spherized.urdf`；开启后关节限位/自碰撞走 QP-CBF |
| `cbf_config.enable_self_collision` | `false` | 有 spherized URDF 后可开 |

仿真阶段通常保持 `false`，先用 `redundant_ik_safety` 即可。

### 2.7 后处理与 MIT（§7）

| 参数 | 默认 | 调参建议 |
|------|------|----------|
| `post_cbf_joint_reference_filter.enable` | `true` | 关节目标低通；MIT 抖时可 ↓ `position_new_weight`（如 0.02） |
| `post_cbf_joint_reference_filter.position_new_weight` | `0.05` | 越小滤波越强、越平滑、越迟滞 |
| `mit_cmd_velocity_zero` | `true` | 一般保持 `true`，速度项由位置误差隐含 |
| `motion_planning_type` | `2` | 五次多项式；复位段平滑性不足可保持 2 |

### 2.8 控制增益（§8）— 下发 MIT 的关键

| 参数 | 默认 | 调参建议 |
|------|------|----------|
| `motion_planning_kp` | `[100,80,80,80, 40,40,40]` | **刚度**：仿真抖 ↓ 近端/腕部 kp；跟手软 ↑ 腕部 kp（注意力矩饱和） |
| `motion_planning_kd` | `[10,8,8,8, 4,4,4]` | **阻尼**：抖 ↑ kd；粘滞 ↑ 略减 kd |
| `gravity_comp_kd` | 见 yaml | 仿真重力补偿；实机通常接近 0 |
| `damping_control_kp/kd` | 见 yaml | 空闲/故障持位；非遥操态手感 |
| `enable_integral_control` | `false` | 稳态误差大再开；注意 `integral_limit` / `output_limit` 防 windup |

实机须对照 Franka 各关节 `effort` 上限，避免 `motion_planning_kp` 过大导致保护停。

### 2.9 外力估计与力反馈（§9）

| 参数 | 默认 | 调参建议 |
|------|------|----------|
| `force_estimation_method` | `momentum_observer` | 仿真可用；**实机推荐 `delta_momentum`**（无积分漂移） |
| `dynamics_force_estimator.momentum_observer_gain` | `7.0` | ↑ 响应快、噪声大；↓ 更平滑 |
| `dynamics_force_estimator.output_filter_cutoff_hz` | `5.0` | 输出力低通；手感噪 ↓ 截止频率 |
| `ee_ff_gain` | `0.0` | 从臂侧保持 0（只估计发布）；**主臂侧**在 `master_arm_control_nexus_v15_right.yaml` 调 |
| `force_publish_lower_limit` | `[10,10,10, …]` N | 力反馈死区；太小易噪，太大无感 |
| `force_publish_lpf_alpha` | 见 yaml | 发布前低通；<1 更平滑 |
| `ik_boundary_force.enable` | `true` | 从臂 IK 连续失败时虚拟弹簧推主臂回可达区 |
| `ik_boundary_force.boundary_kp/kd` | 见 yaml | 边界力刚度/阻尼；推回过猛 ↓ kp |

---

## 3. gripper_config_fr3v2_1_hand.yaml

从臂第 8 关节 `fr3v2_1_finger_joint1`：**prismatic，单位 m**，范围 0（闭合）～ 0.04（全开）。

数据流：

```
manage (gripper_scaling_factor) → controller 归一化 → finger_joint1 目标 (m) → MIT (kp/kd) → mujoco_sim / teleop_adapter
```

### 3.1 行程与角色

| 参数 | 默认 | 说明与建议 |
|------|------|------------|
| `self_role` | `slave` | 必须为 `slave` |
| `slave_max_opening_distance` | `40.0` mm | 对应 URDF upper=0.04 m；**勿与 Y1 表混用** |
| `slave_min_opening_distance` | `0.0` | 闭合端 |
| `master_max_opening_distance` | `20.58` mm | 主臂行程表最大位移，用于力反馈归一化 |

重算主从比例：

`gripper_scaling_factor = slave_max_opening_distance / master_max_opening_distance`。具体值见 `gripper_config_fr3v2_1_hand.yaml` 中的 `slave_max_opening_distance` 和 `master_max_opening_distance`。

### 3.2 MIT 控制（仿真直接消费）

| 参数 | 默认 | 调参建议 |
|------|------|----------|
| `kp` | `500.0` | **夹爪刚度**：闭合抖/振荡 ↓（如 100～300）；闭合慢、跟不上 ↑ |
| `kd` | `0.3` | 阻尼；与 `kp` 同向调节 |
| `target_angle_filter_alpha` | `1.0` | 目标开距低通；抖 ↓ alpha（如 0.5）；跟手 ↑  toward 1.0 |
| `max_gripper_delta` | `0.01` m | 每步最大开距变化（10 mm）；突变大 ↓（如 0.005） |
| `max_gripper_delta_velocity_threshold` | `0.15` m/s | 仅低速时启用步进限幅；快速开合时被绕过 |
| `kd_velocity_limit` | `0.2` m/s | kd 项速度误差饱和 |

### 3.3 夹爪力反馈 FSM（从臂计算 gripper_force）

| 参数 | 默认 | 调参建议 |
|------|------|----------|
| `force_feedback_use_fsm` | `false` | 仿真/首版关闭；实机主从归一化对齐后再开 |
| `ff_x_on` | `0.08` | 进入 HOLDING 的归一化差阈值 |
| `ff_force_lower_limit` / `ff_force_upper_limit` | `0.2` / `0.4` N | 输出力幅值；主臂 `force_feedback_gain` 配合 |
| `ff_slave_velocity_threshold` | `0.05` m/s | 从臂夹爪速度门控（Franka 为 m/s 标定） |

---

## 4. nexus_manage 主从映射

文件：`nexus_manage/config/nexus-arm_v15_right_to_fr3v2_1_hand_manage.yaml`

### 4.1 复位姿态（最先标定）

| 块 | 参数 | 说明 |
|----|------|------|
| `right_wrist` | `teleop_pos_values` | 主臂复位关节角（6 轴） |
| `right_wrist` | `robot_pos_values` | 从臂复位关节角（7 轴），须与 Franka home 一致 |
| `right_end_effector` | `teleop_pos_values: [0.0]` | 主臂夹爪全开（`gripper_Joint=0` rad） |
| `right_end_effector` | `robot_pos_values: [0.04]` | 从臂夹爪全开（m） |

复位不对时，优先改此处，**不要先动 controller 增益**。

### 4.2 末端与夹爪缩放

| 参数 | 默认 | 调参建议 |
|------|------|----------|
| `ee_pos_scaling_factor_x/y/z` | `1.0` | 主从工作空间比例；某轴行程不够时微调 |
| `ee_rot_scaling_factor` | `1.0` | 姿态缩放，一般保持 1.0 |
| `gripper_scaling_factor` | `0.063694` | 主臂无量纲 → 从臂米制；**夹爪行程比例不对时改此值** |
| `gripper_sign_flip_enabled` | `false` | 开合方向反了再改 `true` |
| `ee_rpy_limits` | 见 yaml | 主臂姿态增量限幅 |

---

## 5. 主臂与其它配置

### 5.1 主臂臂控 `master_arm_control_nexus_v15_right.yaml`

| 参数 | 调参建议 |
|------|----------|
| `ee_ff_gain` | 主臂力反馈总增益；从臂估计稳定后 0→小值逐步加 |
| `nexus_friction.*` | 主臂拖动摩擦补偿；实机拖动手感 |
| `force_feedback_extra_kd` | 有力反馈时附加关节阻尼，抑振 |
| `gravity_comp_kd` | TELEOP 态拖动阻尼 |

### 5.2 主臂夹爪 `gripper_config_nexus_v15_right.yaml`

| 参数 | 说明 |
|------|------|
| `master_stroke_table_path` | V15 Right 行程表 |
| `slave_max_opening_distance: 40.0` | Franka 从臂行程，用于主臂侧力反馈归一化 |
| `force_feedback_gain` | 主臂接收从臂 `gripper_force` 的缩放 |

### 5.3 从臂结构 `slave_single_fr3v2_1_hand_config.yaml`

| 参数 | 说明 |
|------|------|
| `is_simulation` | 与 `arm_control_fr3v2_1_hand.yaml` 中一致；实机均为 `false` |
| `robot_type` | `franka_hand`，影响 GripperController 映射分支 |

### 5.4 MuJoCo `fr3v2_1_hand_mujoco_sim.yaml`

| 参数 | 说明 |
|------|------|
| `publish_rate` | 关节状态发布频率 |
| `enable_viewer` | 无显示器时 `false` |

---

## 6. 现象 → 参数对照

| 现象 | 优先检查 |
|------|----------|
| 复位后主从姿态/夹爪不一致 | manage `teleop_pos_values` / `robot_pos_values` |
| 末端跟手慢、明显滞后 | ↑ `vqp_clik_gain`；↑ `motion_planning_kp` |
| 腕部或全臂抖动 | ↓ `motion_planning_kp`；↑ `motion_planning_kd`；↓ `vqp_max_total_joint_change`；开/加强 `post_cbf_joint_reference_filter` |
| 肘部姿态慢慢漂移 | ↑ `vqp_arm_angle_weight`；↑ `vqp_ref_joint_weight` |
| 接近奇异位形突变 | 确认 `vqp_adaptive_reg_enable`；↓ `vqp_clik_gain`；缩 `redundant_ik_safety` 工作空间 |
| 关节蹭限位 | ↑ `vqp_position_limit_gain`；核对 `redundant_ik_safety.joint_limit` |
| 夹爪主从行程比例不对 | `gripper_scaling_factor`；核对 `slave_max_opening_distance` / `master_max_opening_distance` |
| 夹爪闭合抖 | ↓ `gripper_config` 的 `kp`；↓ `max_gripper_delta` |
| 夹爪闭合慢 | ↑ `kp`；↑ `target_angle_filter_alpha` |
| 主臂感受不到接触力 | 从臂 `force_estimation_method`；↑ 主臂 `ee_ff_gain`；检查 `force_publish_lower_limit` |
| IK 不可达时主臂“顶手” | `ik_boundary_force.boundary_kp/kd` |
| 实机保护停 / 力矩过大 | ↓ `motion_planning_kp`；核对 FR3 effort 限位；`is_simulation: false` |

---

*文档版本：2026-06 — 对应 `arm_control_fr3v2_1_hand.yaml` / `gripper_config_fr3v2_1_hand.yaml` 当前默认值。*
