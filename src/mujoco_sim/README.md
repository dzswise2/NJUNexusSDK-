# MuJoCo Simulator (mujoco_sim)

基于 MuJoCo 物理引擎的 ROS2 机器人仿真功能包，提供高精度的动力学仿真环境。

## 功能特性

- 🎮 MuJoCo 物理引擎仿真
- 🤖 多机器人模型支持
- 🔄 ROS2 话题接口（关节状态发布、关节控制接收）
- ⚙️ URDF 到 MJCF 自动转换工具
- 📊 可配置的关节摩擦/阻尼参数

## 支持的机器人

| 机器人 | Launch 文件 | 配置文件 | 说明 |
|--------|-------------|----------|------|
| Y1 (单臂) | `y1_sim.launch.py` | `mujoco_sim.yaml` | Y1 机械臂单臂仿真 |
| Y1 (双臂) | `y1_dual_sim.launch.py` | `y1_master/slave_mujoco_sim.yaml` | Y1 主从双臂仿真 |
| Nexus-Arm | `nexus_arm_sim.launch.py` | `nexus_arm_mujoco_sim.yaml` | Nexus 机械臂仿真 |
| Piper | `piper_sim.launch.py` | `piper_mujoco_sim.yaml` | Piper 机械臂仿真 |

## 快速开始

### 1. 编译功能包

```bash
cd ~/nexus-sdk
colcon build --packages-select mujoco_sim
source install/setup.bash
```

### 2. 启动仿真

**Y1 单臂仿真：**
```bash
ros2 launch mujoco_sim y1_sim.launch.py
```

**Y1 双臂仿真：**
```bash
ros2 launch mujoco_sim y1_dual_sim.launch.py
```

**Nexus-Arm 仿真：**
```bash
ros2 launch mujoco_sim nexus_arm_sim.launch.py
```

**Piper 仿真：**
```bash
ros2 launch mujoco_sim piper_sim.launch.py
```

## ROS2 话题接口

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/teleop/joint_states` | `sensor_msgs/JointState` | 发布 | 关节状态 |
| `/teleop/joint_cmd` | `std_msgs/Float32MultiArray` | 订阅 | 关节控制指令 |
| `/teleop/arm_status` | `std_msgs/Int32` | 发布 | 机械臂状态 |
| `/teleop/end_pose` | `geometry_msgs/PoseStamped` | 发布 | 末端位姿 |

> 注：具体话题名称可在配置文件中自定义

## 添加新机器人

### 步骤 1：准备 URDF 和 Mesh 文件

在 `robot_description` 目录下创建机器人描述文件夹：

```
robot_description/
└── <robot_name>_description/
    ├── urdf/
    │   └── <robot_name>.urdf
    └── meshes/
        ├── base_link.STL
        ├── link1.STL
        └── ...
```

**URDF 要求：**
- Mesh 路径使用相对路径：`filename="../meshes/xxx.STL"`
- 关节必须包含 `<limit>` 标签（lower, upper, effort, velocity）

### 步骤 2：使用脚本转换 URDF 到 MJCF

```bash
# 首次转换（自动生成配置文件）
python3 src/mujoco_sim/scripts/urdf_to_mjcf.py \
  src/mujoco_sim/robot_description/<robot_name>_description/urdf/<robot_name>.urdf
```

**生成的文件：**
```
robot_description/<robot_name>_description/mjcf/
├── <robot_name>.xml           # 包装器 MJCF（用于仿真）
├── <robot_name>_base.xml      # 纯机器人模型
└── <robot_name>_config.yaml   # 执行器和关节配置
```

### 步骤 3：调整配置参数（可选）

编辑生成的 `<robot_name>_config.yaml`：

```yaml
actuators:
  joint1:
    type: motor          # motor(力矩控制) | position(位置伺服)
    gear: 1              # 齿轮比
    ctrlrange: [-10, 10] # 控制范围
    forcerange: [-10, 10] # 力矩限制

# 关节摩擦参数（自动根据 effort 计算）
joint_properties:
  joint1:
    damping: 0.45        # 粘性摩擦
    frictionloss: 0.09   # 干摩擦
    armature: 0.009      # 转子惯量

scene:
  timestep: 0.002
  gravity: [0, 0, -9.81]
```

修改后重新应用配置：
```bash
python3 src/mujoco_sim/scripts/urdf_to_mjcf.py \
  src/mujoco_sim/robot_description/<robot_name>_description/urdf/<robot_name>.urdf \
  -c src/mujoco_sim/robot_description/<robot_name>_description/mjcf/<robot_name>_config.yaml
```

### 步骤 4：创建 ROS2 配置文件

在 `config/` 目录下创建 `<robot_name>_mujoco_sim.yaml`：

```yaml
mujoco_simulator:
  ros__parameters:
    # 模型路径（相对于包 share 目录）
    model_path: "robot_description/<robot_name>_description/mjcf/<robot_name>.xml"
    robot_name: "<robot_name>"
    robot_type: "<robot_type>"  # 用于夹爪映射
    
    # 仿真参数
    publish_rate: 500.0
    enable_viewer: true
    
    # ROS2 话题
    joint_mit_control_topic: '/teleop/joint_cmd'
    joint_state_topic: '/teleop/joint_states'
    status_topic: '/teleop/arm_status'
    end_pose_topic: '/teleop/end_pose'
    
    # 夹爪配置
    enable_gripper_mapping: true
    gripper_joint_index: 6
    
    # 末端位姿
    enable_end_pose: false
    ee_site_name: 'ee_site'
```

### 步骤 5：创建 Launch 文件

在 `launch/` 目录下创建 `<robot_name>_sim.launch.py`：

```python
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('mujoco_sim')
    config_file = os.path.join(pkg_share, 'config', '<robot_name>_mujoco_sim.yaml')

    simulator_node = Node(
        package='mujoco_sim',
        executable='simulator',
        name='mujoco_simulator',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([simulator_node])
```

### 步骤 6：更新 CMakeLists.txt

确保新文件被安装：

```cmake
install(DIRECTORY
  config
  launch
  robot_description
  DESTINATION share/${PROJECT_NAME}
)
```

### 步骤 7：编译并测试

```bash
colcon build --packages-select mujoco_sim
source install/setup.bash
ros2 launch mujoco_sim <robot_name>_sim.launch.py
```

## 目录结构

```
mujoco_sim/
├── config/                    # ROS2 节点配置文件
│   ├── mujoco_sim.yaml
│   ├── nexus_arm_mujoco_sim.yaml
│   └── ...
├── launch/                    # Launch 文件
│   ├── y1_sim.launch.py
│   ├── nexus_arm_sim.launch.py
│   └── ...
├── mujoco_sim/                # Python 源码
│   └── ros2_bridge.py         # ROS2 仿真节点
├── robot_description/         # 机器人模型
│   ├── y1_description/
│   │   ├── urdf/
│   │   ├── mjcf/
│   │   └── meshes/
│   └── nexus-arm_description/
│       └── ...
└── scripts/                   # 工具脚本
    ├── urdf_to_mjcf.py        # URDF→MJCF 转换工具
    └── README.md              # 脚本使用说明
```

## 关节摩擦参数说明

转换脚本会根据 URDF 中的 `effort` 限制自动计算摩擦参数：

| 参数 | 公式 | 说明 |
|------|------|------|
| `damping` | effort × 0.05 | 粘性摩擦（速度相关） |
| `frictionloss` | effort × 0.01 | 干摩擦（恒定） |
| `armature` | effort × 0.001 | 转子惯量 |

**调优建议：**
- 机器人抖动 → 增加 `damping`
- 关节停不住 → 增加 `frictionloss`
- 运动不平滑 → 增加 `armature`

## 夹爪映射系统

### 为什么需要夹爪映射？

许多机器人的夹爪在仿真和真实硬件中使用不同的关节类型：

| 场景 | 仿真 (MuJoCo) | 真实机器人 |
|------|---------------|------------|
| Y1 | 滑动关节 (stroke: -0.025~0m) | 旋转关节 (angle: 0~1.13rad) |
| Nexus-Arm | 滑动关节 (stroke: 0~0.03m) | 旋转关节 (angle: 0~1.467rad) |
| Piper | 滑动关节 | 滑动关节（无需转换） |

夹爪映射系统负责在仿真和真实机器人之间进行位置、速度、力矩的双向转换。

### 配置文件参数

在 `config/<robot>_mujoco_sim.yaml` 中配置：

```yaml
mujoco_simulator:
  ros__parameters:
    # 机器人类型（决定使用哪个夹爪映射器）
    robot_type: "y1"              # 可选: y1, nexus_arm, piper, generic
    
    # 是否启用夹爪映射
    enable_gripper_mapping: true  # true: 启用转换, false: 直接透传
    
    # 夹爪关节在 joint_states 中的索引（0-based）
    gripper_joint_index: 6        # 第7个关节是夹爪
```

### 支持的机器人类型

| robot_type | 映射器类 | 位置转换 | 速度转换 | 力矩转换 |
|------------|----------|----------|----------|----------|
| `y1`, `y1_master`, `y1_slave` | `Y1GripperMapper` | 查找表插值 | 返回 0 | 取反 |
| `nexus-arm`, `nexus_arm` | `NexusArmGripperMapper` | 线性映射 | 返回 0 | 透传 |
| `piper`, `generic`, `default` | `IdentityGripperMapper` | 透传 | 透传 | 透传 |

### 转换方向说明

```
                    publish_joint_states()
仿真 (MuJoCo)  ─────────────────────────────────►  ROS2 话题
   stroke      stroke_to_angle()                    angle
   velocity    velocity_sim_to_real()               velocity
   effort      effort_sim_to_real()                 effort

                    compute_control()
仿真 (MuJoCo)  ◄─────────────────────────────────  ROS2 话题
   stroke      angle_to_stroke()                    angle
   velocity    velocity_real_to_sim()               velocity
   effort      effort_real_to_sim()                 effort
```

### 添加新的夹爪映射器

如果你的机器人需要自定义的夹爪映射逻辑，需要在 `mujoco_sim/ros2_bridge.py` 中：

**步骤 1：创建映射器类**

```python
class MyRobotGripperMapper(GripperMapperBase):
    """自定义机器人的夹爪映射器"""
    
    # 定义范围常量
    STROKE_MIN = 0.0
    STROKE_MAX = 0.05
    ANGLE_MIN = 0.0
    ANGLE_MAX = 1.57
    
    # ===== 位置转换（必须实现）=====
    
    def stroke_to_angle(self, stroke: float) -> float:
        """仿真 → 真实机器人"""
        # 线性映射示例
        t = (stroke - self.STROKE_MIN) / (self.STROKE_MAX - self.STROKE_MIN)
        t = max(0.0, min(1.0, t))  # 限制在 [0, 1]
        return self.ANGLE_MIN + t * (self.ANGLE_MAX - self.ANGLE_MIN)
    
    def angle_to_stroke(self, angle: float) -> float:
        """真实机器人 → 仿真"""
        t = (angle - self.ANGLE_MIN) / (self.ANGLE_MAX - self.ANGLE_MIN)
        t = max(0.0, min(1.0, t))
        return self.STROKE_MIN + t * (self.STROKE_MAX - self.STROKE_MIN)
    
    # ===== 速度转换（可选，默认透传）=====
    
    def velocity_sim_to_real(self, velocity: float) -> float:
        return 0.0  # 或自定义逻辑
    
    def velocity_real_to_sim(self, velocity: float) -> float:
        return 0.0
    
    # ===== 力矩转换（可选，默认透传）=====
    
    def effort_sim_to_real(self, effort: float) -> float:
        return effort  # 透传
    
    def effort_real_to_sim(self, effort: float) -> float:
        return effort
```

**步骤 2：注册到工厂**

在 `GripperMapperFactory.ROBOT_MAPPERS` 中添加：

```python
class GripperMapperFactory:
    ROBOT_MAPPERS = {
        'y1': Y1GripperMapper,
        'nexus_arm': NexusArmGripperMapper,
        'piper': IdentityGripperMapper,
        # 添加你的机器人
        'my_robot': MyRobotGripperMapper,
    }
```

**步骤 3：在配置文件中使用**

```yaml
mujoco_simulator:
  ros__parameters:
    robot_type: "my_robot"
    enable_gripper_mapping: true
    gripper_joint_index: 6
```

### 禁用夹爪映射

如果不需要夹爪映射（仿真和真实机器人使用相同关节类型）：

```yaml
enable_gripper_mapping: false
```

此时夹爪关节数据将直接透传，不做任何转换。

## 常见问题

### Q: MuJoCo 查看器没有显示？
检查配置文件中 `enable_viewer: true` 是否设置。

### Q: 关节运动范围不对？
检查 URDF 中的 `<limit lower="..." upper="..."/>` 是否正确。

### Q: 夹爪控制不响应？
1. 检查 `gripper_joint_index` 是否正确（从 0 开始计数）
2. 检查 `robot_type` 是否与注册的映射器匹配
3. 检查 `enable_gripper_mapping` 是否为 `true`

### Q: 夹爪位置与真实机器人不匹配？
检查映射器的 `STROKE_MIN/MAX` 和 `ANGLE_MIN/MAX` 是否与实际机器人参数一致。

### Q: 如何调试夹爪映射？
在 `ros2_bridge.py` 中添加日志：
```python
self.get_logger().info(f"Gripper: stroke={stroke:.4f} -> angle={angle:.4f}")
```

## 更多文档

- URDF→MJCF 转换工具详细说明：[scripts/README.md](scripts/README.md)
