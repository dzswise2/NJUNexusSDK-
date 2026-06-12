# URDF to MJCF 转换工具使用说明

## 概述

`urdf_to_mjcf.py` 是一个自动化工具,用于将ROS URDF文件转换为MuJoCo MJCF格式,并自动添加执行器、场景配置等必要组件。

### 转换策略

采用**分离式转换(Split Mode)**:
1. **Base MJCF**: URDF完全转换为纯MJCF(包含机器人结构,无执行器)
2. **Wrapper MJCF**: 包含base文件引用 + 执行器定义 + 场景配置
3. **Config YAML**: 可编辑的执行器参数配置文件

这种方式避免了MuJoCo URDF解析器的bug,同时保持模块化和易维护性。

---

## 快速开始

### 基本用法

```bash
python3 src/mujoco_sim/scripts/urdf_to_mjcf.py \
  src/mujoco_sim/robot_description/<robot_name>/urdf/<robot>.urdf
```

### 示例: 转换Y1机械臂

```bash
cd /path/to/sim-gather
python3 src/mujoco_sim/scripts/urdf_to_mjcf.py \
  src/mujoco_sim/robot_description/y1_description/urdf/y1_with_gripper.urdf
```

**输出:**
```
📖 Parsing URDF: src/mujoco_sim/robot_description/y1_description/urdf/y1_with_gripper.urdf
   Found 8 movable joints
✅ Generated actuator config: .../mjcf/y1_with_gripper_config.yaml
🔨 Step 1: Converting URDF to base MJCF...
✅ Converted URDF to base MJCF: .../mjcf/y1_with_gripper_base.xml
🔨 Step 2: Generating wrapper with actuators...
✅ Generated wrapper MJCF: .../mjcf/y1_with_gripper.xml
✅ Conversion complete!
```

**自动生成的文件:**
- `y1_with_gripper_base.xml` - 基础MJCF(纯机器人结构,无执行器)
- `y1_with_gripper.xml` - 包装器MJCF(包含base引用+执行器+场景,**用于仿真**)
- `y1_with_gripper_config.yaml` - 执行器配置(自动生成,**可手动编辑调优**)

---

## 命令行参数

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `urdf_path` | URDF文件路径(必需) | - |
| `-o, --output` | 输出wrapper MJCF路径 | `<urdf_name>.xml` |
| `-c, --config` | 配置文件路径 | `<urdf_name>_config.yaml` |
| `--no-scene` | 不添加场景元素(地面、光照) | false |

### 高级用法

**自定义输出路径:**
```bash
python3 urdf_to_mjcf.py robot.urdf -o /path/to/output.xml
```

**使用已手动编辑的配置文件(跳过自动生成):**
```bash
# 脚本会加载现有配置,不会覆盖
python3 urdf_to_mjcf.py robot.urdf -c custom_config.yaml
```

**不添加场景(仅机器人):**
```bash
python3 urdf_to_mjcf.py robot.urdf --no-scene
```

---

## 添加新机器人的完整流程

### 1. 准备URDF文件

创建标准的ROS2机器人描述目录结构:

```
src/mujoco_sim/robot_description/
└── <robot_name>_description/
    ├── urdf/
    │   └── <robot_name>.urdf          # 机器人URDF文件
    ├── meshes/
    │   ├── link1.STL                  # 网格文件
    │   ├── link2.STL
    │   └── ...
    └── mjcf/                          # 转换后的MJCF(自动生成)
        ├── <robot_name>_base.xml
        ├── <robot_name>.xml
        └── <robot_name>_config.yaml
```

### 2. URDF要求和规范

#### ✅ URDF必须满足:

1. **Mesh路径格式:**
   ```xml
   <mesh filename="../meshes/link_name.STL"/>
   ```
   - 使用相对路径 `../meshes/`
   - 文件扩展名: `.STL`, `.stl`, `.obj`, `.dae`

2. **关节定义:**
   ```xml
   <joint name="joint1" type="revolute">
     <limit lower="-3.14" upper="3.14" effort="100" velocity="10"/>
   </joint>
   ```
   - 必须包含 `name` 和 `type`
   - 移动关节(`revolute`, `prismatic`)需要 `<limit>` 标签

3. **惯性参数:**
   ```xml
   <inertial>
     <mass value="1.0"/>
     <inertia ixx="0.001" ixy="0" ixz="0" 
              iyy="0.001" iyz="0" izz="0.001"/>
   </inertial>
   ```
   - 每个link应有合理的质量和惯性

#### ⚠️ 已知限制:

- **Mimic关节**: 会被忽略,需要手动添加equality约束
- **传输系统**: URDF的transmission标签会被忽略
- **插件**: Gazebo插件不会被转换

### 3. 首次运行转换(自动生成配置)

**第一次转换时,脚本会自动完成以下操作:**
1. 解析URDF中的所有可动关节
2. 根据关节类型自动生成默认执行器配置(`config.yaml`)
3. 转换URDF为纯MJCF(`base.xml`)
4. 使用配置生成包装器MJCF(`robot.xml`)

```bash
# 首次运行 - 自动生成所有文件
python3 src/mujoco_sim/scripts/urdf_to_mjcf.py \
  src/mujoco_sim/robot_description/<robot_name>_description/urdf/<robot_name>.urdf
```

**生成的文件:**
- `mjcf/<robot_name>_base.xml` - 纯MJCF机器人模型
- `mjcf/<robot_name>.xml` - 包装器(用于仿真)
- `mjcf/<robot_name>_config.yaml` - **自动生成的配置文件**

### 4. 调优执行器参数(可选但推荐)

**步骤4.1: 编辑自动生成的配置文件**

打开 `mjcf/<robot_name>_config.yaml` 并根据实际需求调整:

```yaml
actuators:
  joint1:
    type: position        # 类型: position(位置伺服) | motor(力矩) | velocity(速度)
    gear: 100            # 齿轮比(影响力矩放大)
    kp: 100              # 位置增益(仅position类型有效,值越大响应越快)
    ctrlrange: [-3.14, 3.14]   # 控制输入范围: position类型=目标位置, motor类型=力矩, velocity类型=目标速度
    forcerange: [-100.0, 100.0] # 执行器输出力/力矩的限制
  joint2:
    type: motor          # motor类型不使用kp参数
    gear: 200
    ctrlrange: [-50.0, 50.0]    # motor类型: 控制力矩输入范围
    forcerange: [-50.0, 50.0]   # 执行器输出力矩限制

# 关节摩擦/阻尼参数 (自动生成)
# 这些参数会自动应用到生成的 base.xml 中的每个关节
joint_properties:
  joint1:
    damping: 0.5         # 粘性摩擦/阻尼 (N·m·s/rad 或 N·s/m)
    frictionloss: 0.1    # 干摩擦损失 (N·m 或 N)
    armature: 0.01       # 电机转子惯量 (kg·m² 或 kg)
  joint2:
    damping: 0.2
    frictionloss: 0.05
    armature: 0.005

scene:
  timestep: 0.002        # 仿真时间步长(越小越精确但越慢)
  gravity: [0, 0, -9.81] # 重力向量
  lighting: true         # 启用光照
  ground: true          # 添加地面

# 几何体碰撞/显示属性 (自动生成)
# 控制每个link几何体的碰撞行为和可视化分组
geom_properties:
  base_link:
    contype: 0           # 碰撞类型位掩码 (0 = 不参与碰撞)
    conaffinity: 0       # 碰撞亲和位掩码 (0 = 不与任何几何体碰撞)
    density: 0           # 几何体密度 (0 = 使用URDF惯性,不根据形状重算)
    group: 1             # 可视化分组 (0-5,用于MuJoCo Viewer中按组显示/隐藏)
  link1_Joint:
    contype: 1           # 碰撞类型位掩码 (1 = 类型0)
    conaffinity: 15      # 碰撞亲和位掩码 (15 = 0b1111,与类型0-3均碰撞)
    density: 0           # 使用URDF惯性
    group: 0             # 默认可视化分组
```

**关节摩擦参数说明:**

| 参数 | 说明 | 效果 | 单位 |
|------|------|------|------|
| `damping` | 粘性摩擦(阻尼) | 速度越快阻力越大 | N·m·s/rad (旋转) 或 N·s/m (滑动) |
| `frictionloss` | 干摩擦损失 | 恒定摩擦力,与速度无关 | N·m (旋转) 或 N (滑动) |
| `armature` | 电机转子惯量 | 增加等效惯量,运动更平滑 | kg·m² (旋转) 或 kg (滑动) |

**几何体碰撞属性参数说明:**

| 参数 | 说明 | 取值范围 | 效果 |
|------|------|----------|------|
| `contype` | 碰撞类型位掩码 | 0~2³¹-1 (整数) | 定义该几何体的碰撞"发起类型",0 表示不参与碰撞 |
| `conaffinity` | 碰撞亲和位掩码 | 0~2³¹-1 (整数) | 定义该几何体愿意与哪些类型碰撞 |
| `density` | 几何体密度 (kg/m³) | ≥0 | 0 = 使用 URDF 惯性参数;>0 则根据几何体积自动计算质量 |
| `group` | 可视化分组 | 0~5 | MuJoCo Viewer 中可按组切换显示/隐藏(快捷键 0-5) |

**碰撞判定规则:**

两个几何体 A 和 B 能否碰撞,取决于位掩码按位与:
```
可碰撞 = (A.contype & B.conaffinity) || (B.contype & A.conaffinity)
```
- `contype=0, conaffinity=0` → 完全不碰撞(适合底座等固定部件)
- `contype=1, conaffinity=15(0b1111)` → 与 contype 为 0~3 的几何体均碰撞(适合可动连杆)

**自动生成的默认值:**

| 几何体类型 | contype | conaffinity | density | group |
|-----------|---------|-------------|---------|-------|
| base_link(底座) | 0 | 0 | 0 | 1 |
| 可动连杆 | 1 | 15 | 0 | 0 |

> **提示:** 如果仿真中需要抓取物体等接触交互,确保机器人末端连杆和目标物体的 contype/conaffinity 位掩码有交集。如果不需要碰撞检测(如纯运动学验证),可以将所有连杆的 contype 和 conaffinity 都设为 0。

**自动生成的默认值(基于URDF中的effort限制):**

```
旋转关节 (revolute):
  damping     = effort × 0.05  (5% of max torque, min: 0.05)
  frictionloss = effort × 0.01  (1% of max torque, min: 0.01)
  armature    = effort × 0.001 (0.1% of max torque, min: 0.0005)

滑动关节 (prismatic):
  damping     = effort × 0.1   (10% of max force, min: 0.05)
  frictionloss = effort × 0.02  (2% of max force, min: 0.01)
  armature    = effort × 0.01  (1% of max force, min: 0.0005)
```

**示例**: 如果关节 effort=9 N·m:
- damping = 9 × 0.05 = 0.45
- frictionloss = 9 × 0.01 = 0.09
- armature = 9 × 0.001 = 0.009

**常见调优场景:**
- 机器人抖动/不稳定 → 降低 `kp` 值(如从100降到50), 或增加 `damping`
- 响应太慢 → 增加 `kp` 值或减小 `timestep`
- 力不足 → 增加 `gear` 或 `forcerange`
- 关节太滑/停不住 → 增加 `damping` 和 `frictionloss`
- 运动不平滑 → 增加 `armature`

**步骤4.2: 应用修改后的配置**

```bash
# 使用 -c 参数加载已编辑的配置,重新生成wrapper MJCF
python3 src/mujoco_sim/scripts/urdf_to_mjcf.py \
  src/mujoco_sim/robot_description/<robot_name>_description/urdf/<robot_name>.urdf \
  -c src/mujoco_sim/robot_description/<robot_name>_description/mjcf/<robot_name>_config.yaml
```

**注意:**
- 带 `-c` 参数时,脚本**不会覆盖**你的配置文件
- 只会重新生成 `<robot_name>.xml` 和 `<robot_name>_base.xml`
- 配置文件(`config.yaml`)保持不变

### 5. 创建Launch文件

复制并修改 `y1_sim.launch.py`:

```python
# src/mujoco_sim/launch/<robot_name>_sim.launch.py
#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('mujoco_sim')
    
    # 修改为你的机器人MJCF路径
    robot_mjcf_path = os.path.join(
        pkg_share,
        'robot_description',
        '<robot_name>_description',
        'mjcf',
        '<robot_name>.xml'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='100.0',
        description='Rate for publishing joint states (Hz)'
    )
    
    enable_viewer_arg = DeclareLaunchArgument(
        'enable_viewer',
        default_value='true',
        description='Enable MuJoCo viewer window'
    )
    
    simulator_node = Node(
        package='mujoco_sim',
        executable='simulator',
        name='mujoco_simulator',
        output='screen',
        parameters=[{
            'model_path': robot_mjcf_path,
            'scene_path': '',
            'publish_rate': LaunchConfiguration('publish_rate'),
            'enable_viewer': LaunchConfiguration('enable_viewer'),
        }]
    )
    
    return LaunchDescription([
        publish_rate_arg,
        enable_viewer_arg,
        simulator_node,
    ])
```

### 6. 更新setup.py

编辑 `src/mujoco_sim/setup.py`,添加新的数据文件:

```python
def get_data_files():
    data_files = []
    
    # ... 现有代码 ...
    
    # 添加新机器人的文件
    robot_desc_src = 'robot_description/<robot_name>_description'
    robot_desc_dest = 'share/mujoco_sim/robot_description/<robot_name>_description'
    
    for src_dir in ['urdf', 'meshes', 'mjcf']:
        src_path = os.path.join(robot_desc_src, src_dir)
        if os.path.exists(src_path):
            for root, dirs, files in os.walk(src_path):
                if files:
                    file_list = [os.path.join(root, f) for f in files]
                    rel_root = os.path.relpath(root, robot_desc_src)
                    install_path = os.path.join(robot_desc_dest, rel_root)
                    data_files.append((install_path, file_list))
    
    return data_files
```

### 7. 构建和测试

```bash
# 构建包
cd /path/to/sim-gather
colcon build --packages-select mujoco_sim

# 加载环境
source install/setup.bash

# 启动仿真
ros2 launch mujoco_sim <robot_name>_sim.launch.py
```

---

## 验证和调试

### 验证MJCF文件

使用Python测试加载:

```python
import mujoco

model = mujoco.MjModel.from_xml_path('path/to/robot.xml')
print(f"DOF: {model.nq}")
print(f"Actuators: {model.nu}")
print(f"Bodies: {model.nbody}")
```

### 常见问题

#### 1. "Error opening file 'xxx.STL'"

**原因:** Mesh路径不正确  
**解决:** 
- 检查URDF中mesh路径是否为 `../meshes/xxx.STL`
- 确认STL文件存在于 `meshes/` 目录
- 重新运行转换脚本

#### 2. "No actuators defined"

**原因:** URDF中所有关节都是fixed类型  
**解决:** 检查URDF确保有 `type="revolute"` 或 `type="prismatic"` 关节

#### 3. "Actuator <name> references unknown joint"

**原因:** 配置文件中的关节名与URDF不匹配(可能URDF已修改但配置文件未更新)  
**解决:** 
```bash
# 删除旧配置,让脚本重新自动生成
rm mjcf/<robot>_config.yaml
python3 urdf_to_mjcf.py urdf/<robot>.urdf  # 不带-c参数,会生成新配置
```

#### 4. 仿真不稳定/机器人晃动

**原因:** 执行器增益过高或惯性参数不合理  
**解决:**
- 降低config.yaml中的 `kp` 值
- 检查URDF中的惯性参数是否合理
- 增加仿真时间步长 `timestep: 0.005`

---

## 执行器类型说明

### Position (位置伺服)
- **适用于**: 关节位置控制
- **控制模式**: PD控制器(比例-微分)
- **必需参数**: `kp`(位置增益), `gear`, `ctrlrange`
- **控制输入**: 目标位置(弧度或米)
- **说明**: 使用内置PD控制器跟踪目标位置,kp值决定收敛速度

### Motor (力/力矩电机)
- **适用于**: 直接力矩控制
- **控制模式**: 开环力矩控制
- **必需参数**: `gear`, `ctrlrange`, `forcerange`
- **控制输入**: 力或力矩值(N或N·m)
- **说明**: 不使用kp参数,直接输出控制力矩

### Velocity (速度伺服)
- **适用于**: 速度控制
- **控制模式**: 速度PD控制器
- **必需参数**: `kv`(速度增益), `gear`
- **控制输入**: 目标速度(rad/s或m/s)
- **说明**: 使用速度增益kv跟踪目标速度

---

## 目录结构总结

完整的机器人描述目录:

```
src/mujoco_sim/
├── robot_description/
│   └── <robot_name>_description/
│       ├── urdf/                    # 源文件(版本控制)
│       │   └── <robot>.urdf
│       ├── meshes/                  # 3D网格(版本控制)
│       │   └── *.STL
│       └── mjcf/                    # 生成文件(可选版本控制)
│           ├── <robot>_base.xml     # 基础MJCF
│           ├── <robot>.xml          # 包装器(用于仿真)
│           └── <robot>_config.yaml  # 配置(建议版本控制)
├── launch/
│   └── <robot>_sim.launch.py        # 启动文件
└── scripts/
    └── urdf_to_mjcf.py              # 转换工具
```

**版本控制建议:**
- ✅ **必须**: urdf/, meshes/, config.yaml, launch/
- ⚠️ **可选**: mjcf/*.xml (可从URDF重新生成)
- ❌ **不要**: build/, install/, log/

---

## 高级功能

### 添加传感器

手动编辑wrapper MJCF,在 `</actuator>` 后添加:

```xml
<sensor>
  <force name="ee_force" site="end_effector"/>
  <torque name="ee_torque" site="end_effector"/>
  <accelerometer name="base_imu" site="base_link"/>
</sensor>
```

### 添加Equality约束(Mimic关节)

如果URDF中有mimic关节,手动添加:

```xml
<equality>
  <joint joint1="gripper_left" joint2="gripper_right" polycoef="0 -1 0 0 0"/>
</equality>
```

### 自定义接触参数

在wrapper中修改:

```xml
<option>
  <flag contact="enable"/>
</option>

<contact>
  <pair geom1="finger1" geom2="object" condim="4"/>
</contact>
```

---

## 参考资源

- [MuJoCo文档](https://mujoco.readthedocs.io/)
- [URDF规范](http://wiki.ros.org/urdf/XML)
- [MJCF格式参考](https://mujoco.readthedocs.io/en/stable/XMLreference.html)
- 本项目: `/home/qj00431/infra/sim-gather`

---

## 附录: 完整示例

查看Y1机械臂示例:
```bash
tree src/mujoco_sim/robot_description/y1_description/
```

关键文件:
- URDF源: `urdf/y1_with_gripper.urdf`
- 基础MJCF: `mjcf/y1_with_gripper_base.xml`
- 包装器: `mjcf/y1_with_gripper.xml`
- 配置: `mjcf/y1_with_gripper_config.yaml`
- Launch: `launch/y1_sim.launch.py`
