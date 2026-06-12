#!/usr/bin/env python3
"""Validate AR5 08L + suction cup composed URDF."""

from __future__ import annotations

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ARM_PREFIX = "AR5-5_08L-W4C4A6-ZY2"
ROBOT_DIR = f"{ARM_PREFIX}_with_suction_cup"
URDF_NAME = f"{ARM_PREFIX}_with_suction_cup.urdf"


def _mass(link: ET.Element) -> float | None:
    inertial = link.find("inertial")
    if inertial is None:
        return None
    mass = inertial.find("mass")
    if mass is None:
        return None
    return float(mass.get("value", "nan"))


def main() -> int:
    repo = Path(__file__).resolve().parents[1]
    urdf_path = (
        repo
        / "src"
        / "mujoco_sim"
        / "robot_description"
        / ROBOT_DIR
        / "urdf"
        / URDF_NAME
    )
    if not urdf_path.is_file():
        print(f"FAIL: URDF missing: {urdf_path}")
        return 1

    root = ET.parse(urdf_path).getroot()
    errors: list[str] = []

    revolute = [
        j.get("name")
        for j in root.findall("joint")
        if j.get("type") == "revolute"
    ]
    if len(revolute) != 8:
        errors.append(f"expected 8 revolute joints, got {len(revolute)}: {revolute}")

    link7 = root.find(f'.//link[@name="{ARM_PREFIX}_link7"]')
    if link7 is None:
        errors.append("missing link7")
    else:
        m = _mass(link7)
        if m is None or abs(m - 0.33) > 1e-6:
            errors.append(f"link7 mass must be 0.33, got {m}")

    flan = root.find(f'.//link[@name="{ARM_PREFIX}_flan_link"]')
    if flan is None:
        errors.append("missing flan_link")

    base_link = root.find(f'.//link[@name="{ARM_PREFIX}_link_suction_cup_base"]')
    if base_link is None:
        errors.append("missing link_suction_cup_base")
    else:
        base_mesh = base_link.find('.//mesh')
        if base_mesh is None or "link_suction_cup_base" not in base_mesh.get("filename", ""):
            errors.append("base link missing suction_cup_base mesh")

    base_joint = root.find(f'.//joint[@name="{ARM_PREFIX}_joint_suction_cup_base"]')
    if base_joint is None or base_joint.get("type") != "fixed":
        errors.append("missing fixed joint_suction_cup_base")
    else:
        parent = base_joint.find("parent")
        if parent is None or parent.get("link") != f"{ARM_PREFIX}_flan_link":
            errors.append(
                f"suction base must mount on flan_link, got "
                f"{parent.get('link') if parent is not None else None}"
            )

    suction_joint = root.find(f'.//joint[@name="{ARM_PREFIX}_joint_suction_cup"]')
    if suction_joint is None:
        errors.append("missing suction cup joint")
    else:
        parent = suction_joint.find("parent")
        if parent is None or parent.get("link") != f"{ARM_PREFIX}_link_suction_cup_base":
            errors.append(
                f"suction revolute parent must be link_suction_cup_base, "
                f"got {parent.get('link') if parent is not None else None}"
            )
        limit = suction_joint.find("limit")
        if limit is None:
            errors.append("suction joint missing limit")
        else:
            lower = float(limit.get("lower", "nan"))
            upper = float(limit.get("upper", "nan"))
            if abs(lower + 1.5708) > 1e-4 or abs(upper - 1.5708) > 1e-4:
                errors.append(
                    f"suction joint limits must be ±1.5708 rad, got [{lower}, {upper}]"
                )

    if base_link is not None:
        base_mesh = base_link.find('.//mesh')
        if base_mesh is not None and base_mesh.get("scale"):
            errors.append("suction base mesh must be meters (no scale attribute)")

    cup_link = root.find(f'.//link[@name="{ARM_PREFIX}_link_suction_cup"]')
    if cup_link is not None:
        m = _mass(cup_link)
        if m is None or abs(m - 0.0423726736538326) > 1e-3:
            errors.append(f"cup mass expected ~0.0424 kg, got {m}")
    if base_link is not None:
        m = _mass(base_link)
        if m is None or abs(m - 1.00422732634617) > 1e-3:
            errors.append(f"base mass expected ~1.0042 kg, got {m}")

    for j in revolute:
        if j and not j.startswith(ARM_PREFIX):
            errors.append(f"non-08L joint name: {j}")

    text = urdf_path.read_text(encoding="utf-8")
    if "AR5-5_07L-W4C4A2" in text:
        errors.append("07L prefix leaked into composed URDF")

    try:
        import pinocchio  # type: ignore

        model = pinocchio.buildModelFromUrdf(str(urdf_path))
        if model.nv != 8:
            errors.append(f"Pinocchio nv expected 8, got {model.nv}")
    except ImportError:
        print("WARN: pinocchio not installed, skip dynamics load check")
    except Exception as exc:
        errors.append(f"Pinocchio load failed: {exc}")

    if errors:
        for e in errors:
            print(f"FAIL: {e}")
        return 1

    print(f"PASS: {urdf_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
