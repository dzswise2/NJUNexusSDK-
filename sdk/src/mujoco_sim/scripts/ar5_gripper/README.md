# AR5 End-Effector Test Scripts

## test_ar5_gripper.py

夹爪开合测试 — 输入 stroke 在 MuJoCo viewer 中显示构型。

```bash
python3 scripts/ar5_gripper/test_ar5_gripper.py [STROKE] [-H] [-M MAX]
  STROKE  总开距 (m, 0~0.11).  省略则交互输入.
  -H      无 viewer, 仅打印 4 个关节角度.
  -M MAX  最大开距 (m, 默认 0.11 = 110mm).

python3 scripts/ar5_gripper/test_ar5_gripper.py 0.055       # viewer, 55mm
python3 scripts/ar5_gripper/test_ar5_gripper.py 0.055 -H    # headless
```

键盘: `← →` ±2mm  `↑ ↓` ±10mm  `0` 闭合  `f` 全开

## test_ar5_dataflow.py

夹爪数据流闭环 — 模拟 command→state 路径, 打印每一步中间变量。

```bash
python3 scripts/ar5_gripper/test_ar5_dataflow.py [STROKE] [-H]

python3 scripts/ar5_gripper/test_ar5_dataflow.py 0.055 -H   # 打印完整追踪
python3 scripts/ar5_gripper/test_ar5_dataflow.py 0.055      # viewer, 按 p 打印
```

键盘: 同上 + `p` 打印当前数据流追踪.

## test_ar5_suction_cup.py

吸盘转动测试 — 输入角度在 viewer 中显示, 打印数据流追踪。

```bash
python3 scripts/ar5_gripper/test_ar5_suction_cup.py [ANGLE] [-H]
  ANGLE  目标角度 (°, -90~90).  省略则交互输入.

python3 scripts/ar5_gripper/test_ar5_suction_cup.py 45      # viewer, 45°
python3 scripts/ar5_gripper/test_ar5_suction_cup.py -90 -H  # headless
```

键盘: `← →` ±2°  `↑ ↓` ±10°  `0` 归零  `f` +90°  `r` -90°  `p` 打印追踪

---

## 数据流对比

| | 夹爪 | 吸盘 |
|------|------|------|
| CMD ctrl[7] | 总开距 (m) | 角度 (rad) |
| STATE pos[7] | 总开距 (m) | 角度 (rad) |
| Mapper | AR5GripperMapper (公式) | IdentityGripperMapper (直通) |
| 执行器类型 | position | motor |
| 额外约束 | 3 equality (四杆联动) | 无 |
