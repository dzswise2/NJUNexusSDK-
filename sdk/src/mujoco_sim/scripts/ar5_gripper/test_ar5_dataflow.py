#!/usr/bin/env python3
"""
AR5 gripper dataflow test — full command→state roundtrip.

Usage:
    python3 scripts/test_ar5_dataflow.py [STROKE]      # viewer mode
    python3 scripts/test_ar5_dataflow.py [STROKE] -H   # headless: print only

    STROKE = total opening in meters (0 ~ 0.11).  Omit for interactive prompt.

Traces the exact same data path as the ROS2 simulation:
  COMMAND  real stroke → real_to_sim_pos() → sim angle → MuJoCo
  STATE    MuJoCo qpos → sim_to_real_pos() → real stroke → JointState

Keyboard (viewer): ← → ↑ ↓ adjust stroke  0=close  f=max  ESC=quit
"""

import argparse
import math
import os
import sys
import threading

import mujoco
import mujoco.viewer

# —————————————————————————————  Mapper (same as AR5GripperMapper)  —————————————————————————————

LEG = 0.055
THETA_0 = math.asin((0.032 - 0.02272) / (2 * LEG))


def stroke_to_angle(stroke: float) -> float:
    """Low-level: real stroke(m) → sim angle(rad)."""
    x = stroke / (2 * LEG) - math.sin(THETA_0)
    return math.asin(max(-1.0, min(1.0, x))) + THETA_0


def angle_to_stroke(angle: float) -> float:
    """Low-level: sim angle(rad) → real stroke(m)."""
    half = LEG * (math.sin(angle - THETA_0) + math.sin(THETA_0))
    return 2.0 * half


def real_to_sim_pos(real_stroke: float) -> float:       # command path
    return stroke_to_angle(real_stroke)


def sim_to_real_pos(sim_angle: float) -> float:          # state path
    return angle_to_stroke(sim_angle)


# —————————————————————————————  Model  —————————————————————————————

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_DIR = os.path.dirname(os.path.dirname(SCRIPT_DIR))
XML = os.path.join(PKG_DIR, 'robot_description',
                   'AR5-5_07L-W4C4A2_with_gripper', 'mjcf',
                   'AR5-5_07L-W4C4A2_with_gripper.xml')


def load():
    model = mujoco.MjModel.from_xml_path(XML)
    data = mujoco.MjData(model)

    g_names = [f'AR5-5_07L-W4C4A2_joint_gripper_{i}' for i in range(1, 5)]
    g_addrs = [int(model.jnt_qposadr[
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)]) for n in g_names]

    arm_addrs = [int(model.jnt_qposadr[
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f'AR5-5_07L-W4C4A2_joint_{i}')
    ]) for i in range(1, 8)]

    return model, data, g_names, g_addrs, arm_addrs


def apply_gripper(data, addrs, theta1):
    for a, v in zip(addrs, (theta1, -theta1, -theta1, theta1)):
        data.qpos[a] = v


# —————————————————————————————  Dataflow tracer  ———————————————————————————————

def trace_dataflow(target_stroke_m: float, model, data, g_names, g_addrs):
    """Execute full command→state roundtrip and print every step."""

    print(f'\n{"=" * 70}')
    print(f'  AR5 Gripper Dataflow Trace')
    print(f'{"=" * 70}')

    # ——— 1. Command input ———
    print(f'\n  ┌─ COMMAND INPUT')
    print(f'  │  target_stroke = {target_stroke_m * 1000:.1f} mm  ({target_stroke_m:.4f} m)')

    # ——— 2. real_to_sim_pos (command path) ———
    sim_angle_cmd = real_to_sim_pos(target_stroke_m)
    print(f'  │')
    print(f'  ├─ real_to_sim_pos({target_stroke_m:.4f})')
    print(f'  │    → stroke_to_angle({target_stroke_m:.4f})')
    print(f'  │      x = {target_stroke_m:.4f}/(2*{LEG}) - sin(θ₀)')
    print(f'  │        = {target_stroke_m/(2*LEG):.4f} - {math.sin(THETA_0):.4f}')
    x_val = target_stroke_m / (2 * LEG) - math.sin(THETA_0)
    print(f'  │        = {x_val:.6f}')
    print(f'  │      θ = asin({max(-1,min(1,x_val)):.6f}) + θ₀')
    print(f'  │        = {math.asin(max(-1,min(1,x_val))):.6f} + {THETA_0:.6f}')
    print(f'  │        = {sim_angle_cmd:.6f} rad  ({math.degrees(sim_angle_cmd):.2f}°)')

    # ——— 3. Apply to MuJoCo ———
    theta1 = sim_angle_cmd
    apply_gripper(data, g_addrs, theta1)
    mujoco.mj_forward(model, data)

    print(f'  │')
    print(f'  ├─ Apply to MuJoCo')
    actual_theta1 = data.qpos[g_addrs[0]]
    print(f'  │    data.qpos[gripper_1] = {theta1:.6f} rad')
    for i in range(4):
        v = data.qpos[g_addrs[i]]
        print(f'  │    data.qpos[gripper_{i+1}] = {v:+.6f} rad  ({math.degrees(v):+.2f}°)')

    # ——— 4. sim_to_real_pos (state path) ———
    actual_stroke = sim_to_real_pos(actual_theta1)
    print(f'  │')
    print(f'  ├─ sim_to_real_pos({actual_theta1:.6f})  ← reads from MuJoCo')
    print(f'  │    → angle_to_stroke({actual_theta1:.6f})')
    print(f'  │      half = {LEG} * (sin({actual_theta1:.6f} - {THETA_0:.6f}) + sin({THETA_0:.6f}))')
    val1 = math.sin(actual_theta1 - THETA_0)
    val2 = math.sin(THETA_0)
    print(f'  │           = {LEG} * ({val1:.6f} + {val2:.6f})')
    half_val = LEG * (val1 + val2)
    print(f'  │           = {half_val:.6f} m')
    print(f'  │      stroke = 2 * {half_val:.6f} = {actual_stroke:.6f} m')

    # ——— 5. State output ———
    print(f'  │')
    print(f'  └─ STATE OUTPUT')
    print(f'       JointState.position[7] = {actual_stroke:.4f} m  =  {actual_stroke*1000:.1f} mm')

    # ——— 6. Roundtrip check ———
    err = abs(target_stroke_m - actual_stroke)
    status = '✓ OK' if err < 1e-9 else f'✗ ERROR ({err:.2e})'
    print(f'\n  ┌─ ROUNDTRIP')
    print(f'  │  in : {target_stroke_m*1000:.1f} mm')
    print(f'  │  out: {actual_stroke*1000:.1f} mm')
    print(f'  │  err: {err:.2e} m  →  {status}')
    print(f'  └─')

    # ——— 7. Joint count ———
    print(f'\n  JointState published: 8 joints')
    print(f'    [0..6] = arm joints 1-7 (rad)')
    print(f'    [7]    = gripper stroke (m)  ←  {actual_stroke*1000:.1f} mm')
    print(f'{"=" * 70}\n')

    return actual_stroke


# —————————————————————————————  main  ———————————————————————————————

def main():
    p = argparse.ArgumentParser()
    p.add_argument('stroke', nargs='?', type=float, default=None)
    p.add_argument('-H', '--headless', action='store_true')
    args = p.parse_args()

    model, data, g_names, g_addrs, arm_addrs = load()
    arm_qpos0 = data.qpos[arm_addrs].copy()

    max_s = 0.11

    if args.stroke is not None:
        stroke = max(0.0, min(max_s, args.stroke))
    else:
        try:
            v = input(f'Target stroke (0 ~ {max_s*1000:.0f} mm) [{max_s*500:.0f}]: ').strip()
            stroke = float(v) if v else max_s / 2
        except (ValueError, EOFError):
            stroke = max_s / 2
        stroke = max(0.0, min(max_s, stroke))

    # Headless: print trace and exit
    if args.headless:
        trace_dataflow(stroke, model, data, g_names, g_addrs)
        return

    # ——— Viewer ———
    lock = threading.Lock()
    shared = [stroke]
    printed = [False]

    def on_key(k):
        s = 0.002
        with lock:
            cur = shared[0]
            if k == 262:       cur = min(max_s, cur + s)       # right
            elif k == 263:     cur = max(0.0, cur - s)         # left
            elif k == 265:     cur = min(max_s, cur + s * 5)   # up
            elif k == 264:     cur = max(0.0, cur - s * 5)     # down
            elif k == 48:      cur = 0.0                       # '0'
            elif k == 70:      cur = max_s                     # 'f'
            elif k == 80:                                      # 'p' — print trace
                trace_dataflow(cur, model, data, g_names, g_addrs)
            shared[0] = cur

    print(f'\n  Target: {stroke*1000:.1f} mm    ← → ±2mm   ↑ ↓ ±10mm')
    print(f'  0=close  f=max({max_s*1000:.0f}mm)  p=print trace  ESC=quit\n')

    with mujoco.viewer.launch_passive(model, data, key_callback=on_key) as viewer:
        while viewer.is_running():
            with lock:
                cur = shared[0]

            data.qpos[arm_addrs] = arm_qpos0
            theta1 = real_to_sim_pos(cur)
            apply_gripper(data, g_addrs, theta1)
            mujoco.mj_forward(model, data)
            viewer.sync()

            if not printed[0]:
                trace_dataflow(cur, model, data, g_names, g_addrs)
                printed[0] = True


if __name__ == '__main__':
    main()
