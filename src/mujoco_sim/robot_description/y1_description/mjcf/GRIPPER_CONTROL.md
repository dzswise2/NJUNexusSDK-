# 夹爪控制说明

## 问题背景
- **真机**：夹爪由 **1个电机** 控制，通过机械耦合同时驱动两个手指
- **仿真**：为了准确模拟两个手指的独立运动，定义了 **2个关节** (joint7, joint8)

## 解决方案
使用 MuJoCo 的 `equality` 约束实现关节耦合，保证：
- 仿真中只有 **7个执行器**（6个机械臂关节 + 1个夹爪）
- 与真机控制接口完全兼容
- joint8 自动跟随 joint7 镜像运动

## 配置详情

### 执行器列表（7个，与真机一致）
```
0. act_joint1  - 关节1（基座旋转）
1. act_joint2  - 关节2（肩部）
2. act_joint3  - 关节3（肘部）
3. act_joint4  - 关节4（腕部1）
4. act_joint5  - 关节5（腕部2）
5. act_joint6  - 关节6（腕部旋转）
6. act_gripper - 夹爪（控制 joint7，joint8 自动跟随）
```

### 耦合关系
```xml
<equality>
  <joint name="gripper_coupling" joint1="joint7" joint2="joint8" polycoef="0 -1 0 0 0" />
</equality>
```
- `polycoef="0 -1 0 0 0"` 表示：`joint2 = -1 * joint1`
- joint7 向负方向移动（-0.05 到 0），joint8 向正方向移动（0 到 0.05）
- 实现镜像对称的夹爪开合

## 算法接口使用

### 控制指令（JointMitControl）
发送 **7维** 控制向量：
```python
msg = JointMitControl()
msg.joint_position = [j1, j2, j3, j4, j5, j6, gripper]  # 7个值
msg.kp = [kp1, kp2, kp3, kp4, kp5, kp6, kp_gripper]      # 7个值
msg.kd = [kd1, kd2, kd3, kd4, kd5, kd6, kd_gripper]      # 7个值
```

### 状态反馈（JointState）
接收 **8维** 状态（包含两个夹爪关节的实际位置）：
```python
# joint_states 包含 8 个关节
# [joint1, joint2, joint3, joint4, joint5, joint6, joint7, joint8]
# 
# 在算法中处理时：
arm_positions = joint_states.position[:6]      # 前6个是机械臂
gripper_position = joint_states.position[6]    # joint7 是夹爪位置
# joint_states.position[7] 是 joint8，因耦合约束会自动等于 -joint7
```

### 夹爪位置说明
- **全开**：`joint7 = 0.0` (joint8 = 0.0)
- **全闭**：`joint7 = -0.05` (joint8 = 0.05)
- **控制范围**：`-0.05 到 0.0`（与真机一致）

## 与真机兼容性
✅ 控制维度一致（7维）
✅ 夹爪控制语义一致（单一值控制开合）
✅ 算法无需区分仿真/真机
✅ 状态反馈包含完整信息（8维，可选择性使用）
