#!/usr/bin/env python3
"""
AR5 suction cup test — drive suction cup joint, display in MuJoCo viewer.

Usage:
    python3 scripts/ar5_gripper/test_ar5_suction_cup.py [ANGLE] [-H]

      ANGLE   Suction cup joint angle in degrees (default: interactive prompt).
      -H      Headless: print dataflow trace and exit.

Examples:
    python3 scripts/ar5_gripper/test_ar5_suction_cup.py 45         # 45° + viewer
    python3 scripts/ar5_gripper/test_ar5_suction_cup.py -90 -H     # -90°, trace only
    python3 scripts/ar5_gripper/test_ar5_suction_cup.py            # prompt + viewer

Keyboard (viewer):
    ← →   fine (±2°)    ↑ ↓   coarse (±10°)
    0     zero position    f     +90°    r     -90°
    ESC   quit
"""

import argparse
import math
import os
import sys
import threading

import mujoco
import mujoco.viewer

# ——— Mapper (IdentityGripperMapper: angle ↔ angle, pass-through) ———

def real_to_sim_pos(angle: float) -> float:
    return angle


def sim_to_real_pos(angle: float) -> float:
    return angle


# ——— Model ———

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_DIR = os.path.dirname(os.path.dirname(SCRIPT_DIR))
XML = os.path.join(PKG_DIR, 'robot_description',
                   'AR5-5_07L-W4C4A2_with_suction_cup', 'mjcf',
                   'AR5-5_07L-W4C4A2_with_suction_cup.xml')


def load():
    model = mujoco.MjModel.from_xml_path(XML)
    data = mujoco.MjData(model)

    sc_name = 'AR5-5_07L-W4C4A2_joint_suction_cup'
    sc_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, sc_name)
    sc_addr = int(model.jnt_qposadr[sc_jid])

    arm_addrs = [int(model.jnt_qposadr[
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f'AR5-5_07L-W4C4A2_joint_{i}')
    ]) for i in range(1, 8)]

    return model, data, sc_name, sc_jid, sc_addr, arm_addrs


# ——— Dataflow tracer ———

def trace_dataflow(target_deg: float, model, data, sc_name, sc_jid, sc_addr):
    target_rad = math.radians(target_deg)
    print(f'\n{"=" * 65}')
    print(f'  AR5 Suction Cup Dataflow Trace')
    print(f'{"=" * 65}')

    # 1. Command input
    print(f'\n  ┌─ COMMAND INPUT')
    print(f'  │  target = {target_deg:.1f}°  ({target_rad:.4f} rad)')

    # 2. real_to_sim_pos (identity pass-through)
    sim_cmd = real_to_sim_pos(target_rad)
    print(f'  │')
    print(f'  ├─ real_to_sim_pos({target_rad:.4f})  [IdentityGripperMapper]')
    print(f'  │    → {sim_cmd:.4f} rad  (angle pass-through)')

    # 3. Apply to MuJoCo
    data.qpos[sc_addr] = sim_cmd
    mujoco.mj_forward(model, data)
    actual = data.qpos[sc_addr]

    print(f'  │')
    print(f'  ├─ Apply to MuJoCo')
    print(f'  │    data.qpos[suction_cup] = {sim_cmd:.4f} rad')
    jrange = model.jnt_range[sc_jid]
    print(f'  │    joint range: [{jrange[0]:.4f}, {jrange[1]:.4f}] rad')
    print(f'  │    limited: {bool(model.jnt_limited[sc_jid])}')

    # 4. sim_to_real_pos (identity pass-through)
    real_out = sim_to_real_pos(actual)
    print(f'  │')
    print(f'  ├─ sim_to_real_pos({actual:.4f})  ← reads from MuJoCo')
    print(f'  │    → {real_out:.4f} rad  ({math.degrees(real_out):.1f}°)')

    # 5. State output
    print(f'  │')
    print(f'  └─ STATE OUTPUT')
    print(f'       JointState.position[7] = {real_out:.4f} rad')
    print(f'       JointState.position[7] = {math.degrees(real_out):.1f}°')

    # 6. Roundtrip
    err = abs(target_rad - real_out)
    status = '✓ OK' if err < 1e-9 else f'✗ ERROR ({err:.2e})'
    print(f'\n  ┌─ ROUNDTRIP')
    print(f'  │  in : {target_deg:.1f}°  ({target_rad:.4f} rad)')
    print(f'  │  out: {math.degrees(real_out):.1f}°  ({real_out:.4f} rad)')
    print(f'  │  err: {err:.2e}  →  {status}')
    print(f'  └─')

    # 7. Joint count
    print(f'\n  JointState published: 8 joints')
    print(f'    [0..6] = arm joints 1-7 (rad)')
    print(f'    [7]    = suction cup angle (rad)  ←  {math.degrees(real_out):.1f}°')
    print(f'{"=" * 65}\n')


# ——— Main ———

def main():
    p = argparse.ArgumentParser()
    p.add_argument('angle', nargs='?', type=float, default=None,
                   help='Suction cup angle in degrees (-90 ~ 90)')
    p.add_argument('-H', '--headless', action='store_true')
    args = p.parse_args()

    model, data, sc_name, sc_jid, sc_addr, arm_addrs = load()
    arm_qpos0 = data.qpos[arm_addrs].copy()

    max_deg = 90.0

    if args.angle is not None:
        angle_deg = max(-max_deg, min(max_deg, args.angle))
    else:
        try:
            v = input(f'Angle ({-max_deg:.0f}° ~ {max_deg:.0f}°) [45]: ').strip()
            angle_deg = float(v) if v else 45.0
        except (ValueError, EOFError):
            angle_deg = 45.0
        angle_deg = max(-max_deg, min(max_deg, angle_deg))

    if args.headless:
        trace_dataflow(angle_deg, model, data, sc_name, sc_jid, sc_addr)
        return

    # ——— Viewer ———
    lock = threading.Lock()
    shared = [angle_deg]
    printed = [False]

    def on_key(k):
        s = 2.0               # fine step
        big = 10.0            # coarse step
        with lock:
            cur = shared[0]
            if k == 262:       cur = min(max_deg, cur + s)          # right
            elif k == 263:     cur = max(-max_deg, cur - s)         # left
            elif k == 265:     cur = min(max_deg, cur + big)        # up
            elif k == 264:     cur = max(-max_deg, cur - big)       # down
            elif k == 48:      cur = 0.0                            # '0'
            elif k == 70:      cur = max_deg                        # 'f'
            elif k == 82:      cur = -max_deg                       # 'r'
            elif k == 80:                                          # 'p'
                trace_dataflow(cur, model, data, sc_name, sc_jid, sc_addr)
            shared[0] = cur

    print(f'\n  Suction Cup — {angle_deg:.1f}°')
    print(f'  ← → ±2°   ↑ ↓ ±10°   0=zero   f=+90°   r=-90°   p=print trace   ESC=quit\n')

    with mujoco.viewer.launch_passive(model, data, key_callback=on_key) as viewer:
        while viewer.is_running():
            with lock:
                cur = shared[0]

            data.qpos[arm_addrs] = arm_qpos0
            sim_angle = real_to_sim_pos(math.radians(cur))
            data.qpos[sc_addr] = sim_angle
            mujoco.mj_forward(model, data)
            viewer.sync()

            if not printed[0]:
                trace_dataflow(cur, model, data, sc_name, sc_jid, sc_addr)
                printed[0] = True


if __name__ == '__main__':
    main()
