#!/usr/bin/env python3
"""
AR5 gripper test — drive gripper via stroke, display in MuJoCo viewer.

Usage:
    python3 scripts/test_ar5_gripper.py [STROKE] [-H] [-M MAX]

      STROKE    Total opening distance in meters (default: interactive prompt).
      -H        Headless: print 4 joint angles and exit (no viewer).
      -M MAX    Maximum stroke in meters (default: 0.055 = 55 mm, real spec).

Examples:
    python3 scripts/test_ar5_gripper.py 0.0275            # 27.5mm + viewer, max 55mm
    python3 scripts/test_ar5_gripper.py 0.11 -M 0.12      # 110mm, extended range
    python3 scripts/test_ar5_gripper.py 0.055 -H          # max open, angles only
    python3 scripts/test_ar5_gripper.py                    # prompt + viewer

Keyboard (viewer):
    ← →   fine open/close  (±2 mm)
    ↑ ↓   coarse (±10 mm)
    0     fully closed
    f     fully open (to MAX)
    ESC   quit
"""

import argparse
import math
import os
import sys
import threading

import mujoco
import mujoco.viewer

# ——————————————————————————————————  kinematics  ——————————————————————————————————

LEG = 0.055
THETA_0 = math.asin((0.032 - 0.02272) / (2 * LEG))


def stroke_to_theta(s: float) -> float:
    x = s / (2 * LEG) - math.sin(THETA_0)
    return math.asin(max(-1.0, min(1.0, x))) + THETA_0


def theta_to_stroke(t: float) -> float:
    return 2 * LEG * (math.sin(t - THETA_0) + math.sin(THETA_0))


def all_angles(theta1: float):
    return theta1, -theta1, -theta1, theta1


# ——————————————————————————————————  model  ——————————————————————————————————

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_DIR = os.path.dirname(os.path.dirname(SCRIPT_DIR))
XML = os.path.join(PKG_DIR, 'robot_description',
                   'AR5-5_07L-W4C4A2_with_gripper', 'mjcf',
                   'AR5-5_07L-W4C4A2_with_gripper.xml')


def load():
    model = mujoco.MjModel.from_xml_path(XML)
    data = mujoco.MjData(model)

    g_names = ['AR5-5_07L-W4C4A2_joint_gripper_%d' % i for i in range(1, 5)]
    g_jids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in g_names]
    g_addr = [int(model.jnt_qposadr[j]) for j in g_jids]

    arm_addr = [int(model.jnt_qposadr[
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,
                          f'AR5-5_07L-W4C4A2_joint_{i}')]) for i in range(1, 8)]

    return model, data, g_names, g_jids, g_addr, arm_addr


# ——————————————————————————————————  helpers  ——————————————————————————————————

def apply_gripper(data, addrs, theta1):
    for a, v in zip(addrs, all_angles(theta1)):
        data.qpos[a] = v


def clamp_stroke(s, max_stroke):
    return max(0.0, min(max_stroke, s))


def print_state(data, model, names, jids, stroke):
    print(f'\n{"=" * 60}')
    print(f'  Stroke          {stroke * 1000:6.1f} mm  (total opening)')
    t1 = data.qpos[int(model.jnt_qposadr[jids[0]])]
    print(f'  θ₁ (gripper_1)  {t1:.4f} rad  ({math.degrees(t1):.2f} deg)')
    print(f'  Roundtrip       {theta_to_stroke(t1) * 1000:.1f} mm')
    print(f'  {"-" * 40}')
    for i in range(4):
        v = data.qpos[int(model.jnt_qposadr[jids[i]])]
        print(f'  gripper_{i + 1}  {v:+.6f} rad   ({math.degrees(v):+.2f} deg)')
    print(f'{"=" * 60}')


# ——————————————————————————————————  main  ——————————————————————————————————

def main():
    p = argparse.ArgumentParser()
    p.add_argument('stroke', nargs='?', type=float, default=None)
    p.add_argument('-H', '--headless', action='store_true')
    p.add_argument('-M', '--max-stroke', type=float, default=0.11,
                   help='Max total opening in meters (default 0.11 = 110 mm, real spec)')
    args = p.parse_args()

    max_stroke = max(0.0, min(0.12, args.max_stroke))

    model, data, g_names, g_jids, g_addr, arm_addr = load()
    arm_qpos0 = data.qpos[arm_addr].copy()

    if args.stroke is not None:
        stroke = clamp_stroke(args.stroke, max_stroke)
    else:
        try:
            v = input(f'Stroke (0 ~ {max_stroke * 1000:.0f} mm) [{max_stroke / 2 * 1000:.0f}]: ').strip()
            stroke = float(v) if v else max_stroke / 2
        except (ValueError, EOFError):
            stroke = max_stroke / 2
        stroke = clamp_stroke(stroke, max_stroke)

    theta1 = stroke_to_theta(stroke)
    apply_gripper(data, g_addr, theta1)
    mujoco.mj_forward(model, data)

    if args.headless:
        print_state(data, model, g_names, g_jids, stroke)
        return

    # ——— viewer ———
    lock = threading.Lock()
    shared_stroke = [stroke]

    def on_key(keycode):
        s = 0.002       # fine step
        big = 0.01      # coarse step
        with lock:
            cur = shared_stroke[0]
            if keycode == 262:          # right
                cur = min(max_stroke, cur + s)
            elif keycode == 263:        # left
                cur = max(0.0, cur - s)
            elif keycode == 265:        # up
                cur = min(max_stroke, cur + big)
            elif keycode == 264:        # down
                cur = max(0.0, cur - big)
            elif keycode == 48:         # '0'
                cur = 0.0
            elif keycode == 70:         # 'f'
                cur = max_stroke
            shared_stroke[0] = cur

    print(f'\nAR5 Gripper — stroke = {stroke * 1000:.1f} mm  (max = {max_stroke * 1000:.0f} mm)')
    print(f'  ← → ±2mm   ↑ ↓ ±10mm   0=close   f=max({max_stroke * 1000:.0f}mm)   ESC=quit')

    printed = False

    with mujoco.viewer.launch_passive(model, data, key_callback=on_key) as viewer:
        while viewer.is_running():
            with lock:
                cur = shared_stroke[0]

            data.qpos[arm_addr] = arm_qpos0
            t1 = stroke_to_theta(cur)
            apply_gripper(data, g_addr, t1)
            mujoco.mj_forward(model, data)
            viewer.sync()

            if not printed:
                print_state(data, model, g_names, g_jids, cur)
                printed = True

    print_state(data, model, g_names, g_jids, shared_stroke[0])


if __name__ == '__main__':
    main()
