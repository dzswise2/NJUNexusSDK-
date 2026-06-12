# MuJoCo Simulation Interactive Testing Tool

用于测试和调试MuJoCo仿真的交互式工具,同时支持发送控制命令和监视关节状态。

## 工具说明

### joint_test - 交互式关节控制与监视工具

集成了控制命令发布和状态监视功能的一体化测试工具。

**功能特性:**
- 同时发布控制命令和监视关节状态
- 多种运行模式(monitor/position/torque/sweep/interactive)
- 多种显示格式(compact/detailed/stats)
- 可配置刷新频率和精度
- 实时反馈机器人行为

**使用方法:**

```bash
# 基础用法 - 仅监视模式(不发送命令)
ros2 run mujoco_sim joint_test --ros-args -p mode:=monitor

# 交互式模式 - 发送位置控制命令并监视状态(默认)
ros2 run mujoco_sim joint_test

# 位置控制模式
ros2 run mujoco_sim joint_test --ros-args -p mode:=position

# 力矩控制模式
ros2 run mujoco_sim joint_test --ros-args -p mode:=torque

# 扫描模式(正弦波轨迹)
ros2 run mujoco_sim joint_test --ros-args -p mode:=sweep
```

**参数说明:**

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `mode` | string | interactive | 运行模式(monitor/interactive/position/torque/sweep) |
| `num_joints` | int | 8 | 机器人关节数量 |
| `publish_rate` | float | 10.0 | 命令发布频率(Hz) |
| `display_rate` | float | 2.0 | 显示刷新频率(Hz) |
| `display_mode` | string | compact | 显示模式(compact/detailed/stats) |
| `precision` | int | 3 | 浮点数精度(小数位数) |

## 典型使用场景

### 场景1: 快速验证机器人响应

```bash
# 终端1: 启动仿真器
ros2 launch mujoco_sim piper_sim.launch.py

# 终端2: 运行交互式测试工具
ros2 run mujoco_sim joint_test
```

### 场景2: 仅监视状态

```bash
ros2 run mujoco_sim joint_test --ros-args -p mode:=monitor
```

### 场景3: 动态轨迹测试

```bash
ros2 run mujoco_sim joint_test --ros-args \
  -p mode:=sweep \
  -p display_mode:=detailed
```

## 许可证

本工具遵循MuJoCo Sim项目的MIT许可证。
