#!/usr/bin/env python3
"""
Import Franka FR3v2.1 + Franka Hand into mujoco_sim/robot_description.

Steps:
  1. Copy meshes from vendor franka_description package
  2. Emit cleaned URDF (no accelerometer virtual links)
  3. Run urdf_to_mjcf.py
  4. Patch wrapper MJCF: 8 actuators + finger mimic equality

Usage:
  python3 prepare_fr3v2_1_hand_sim.py \\
    --franka-src /path/to/franka_description
"""

from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
PKG_ROOT = SCRIPT_DIR.parent
ROBOT_DIR = PKG_ROOT / "robot_description" / "fr3v2_1_franka_hand"
URDF_DIR = ROBOT_DIR / "urdf"
MESH_DIR = ROBOT_DIR / "meshes"
MJCF_DIR = ROBOT_DIR / "mjcf"
URDF_SIM_NAME = "fr3v2_1_franka_hand_sim.urdf"
URDF_CTRL_NAME = "fr3v2_1_franka_hand.urdf"

ACCEL_LINK_RE = re.compile(r"_accelerometer_")


def _indent(elem: ET.Element, level: int = 0) -> None:
    pad = "\n" + "  " * level
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = pad + "  "
        if not elem.tail or not elem.tail.strip():
            elem.tail = pad
        for child in elem:
            _indent(child, level + 1)
        if not child.tail or not child.tail.strip():
            child.tail = pad
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = pad


def _copy_meshes(franka_src: Path) -> None:
    pairs = [
        (franka_src / "meshes" / "robots" / "fr3v2_1", MESH_DIR / "robots" / "fr3v2_1"),
        (franka_src / "meshes" / "robot_ee" / "franka_hand_white",
         MESH_DIR / "robot_ee" / "franka_hand_white"),
    ]
    for src, dst in pairs:
        if not src.is_dir():
            raise SystemExit(f"Missing mesh source: {src}")
        if dst.exists():
            shutil.rmtree(dst)
        shutil.copytree(src, dst)


def _load_cleaned_root(franka_src: Path) -> ET.Element:
    src_urdf = franka_src / "urdfs_relative" / "fr3v2_1_franka_hand.relative.urdf"
    if not src_urdf.is_file():
        raise SystemExit(f"URDF not found: {src_urdf}")

    tree = ET.parse(src_urdf)
    root = tree.getroot()

    remove_links = {
        link.get("name")
        for link in root.findall("link")
        if link.get("name") and ACCEL_LINK_RE.search(link.get("name", ""))
    }
    for link in list(root.findall("link")):
        if link.get("name") in remove_links:
            root.remove(link)

    for joint in list(root.findall("joint")):
        parent = joint.find("parent")
        child = joint.find("child")
        parent_link = parent.get("link") if parent is not None else ""
        child_link = child.get("link") if child is not None else ""
        if (
            joint.get("name", "") in remove_links
            or parent_link in remove_links
            or child_link in remove_links
        ):
            root.remove(joint)

    return root


def _flatten_meshes_for_mujoco() -> None:
    """MuJoCo URDF loader expects flat mesh filenames under meshes/."""
    flat_map = [
        (f"robots/fr3v2_1/collision/link{i}.stl", f"fr3v2_1_link{i}.stl")
        for i in range(8)
    ]
    flat_map.append(
        ("robot_ee/franka_hand_white/collision/hand.stl", "fr3v2_1_hand.stl")
    )
    for rel_src, flat_name in flat_map:
        src = MESH_DIR / rel_src
        dst = MESH_DIR / flat_name
        if not src.is_file():
            raise SystemExit(f"Missing collision mesh: {src}")
        shutil.copy2(src, dst)


def _flatten_sim_urdf_meshes(root: ET.Element) -> None:
    repl = {
        "../meshes/robots/fr3v2_1/visual/": "../meshes/",
        "../meshes/robots/fr3v2_1/collision/": "../meshes/",
        "../meshes/robot_ee/franka_hand_white/visual/hand.dae": "../meshes/fr3v2_1_hand.stl",
        "../meshes/robot_ee/franka_hand_white/collision/hand.stl": "../meshes/fr3v2_1_hand.stl",
        "../meshes/robot_ee/franka_hand_white/visual/finger.dae": "../meshes/fr3v2_1_hand.stl",
    }
    for i in range(8):
        repl[f"../meshes/robots/fr3v2_1/collision/link{i}.stl"] = (
            f"../meshes/fr3v2_1_link{i}.stl"
        )
        repl[f"../meshes/robots/fr3v2_1/visual/link{i}.dae"] = (
            f"../meshes/fr3v2_1_link{i}.stl"
        )
    for mesh in root.iter("mesh"):
        fn = mesh.get("filename", "")
        for old, new in repl.items():
            if fn == old:
                mesh.set("filename", new)
                break


def _write_urdfs(franka_src: Path) -> tuple[Path, Path]:
    URDF_DIR.mkdir(parents=True, exist_ok=True)

    _flatten_meshes_for_mujoco()

    sim_root = _load_cleaned_root(franka_src)
    _flatten_sim_urdf_meshes(sim_root)
    sim_path = URDF_DIR / URDF_SIM_NAME
    _indent(sim_root)
    ET.ElementTree(sim_root).write(sim_path, encoding="utf-8", xml_declaration=True)

    ctrl_root = _load_cleaned_root(franka_src)
    finger2 = None
    for joint in list(ctrl_root.findall("joint")):
        if joint.get("name") == "fr3v2_1_finger_joint2":
            finger2 = joint
            ctrl_root.remove(joint)
    if finger2 is not None:
        fixed = ET.Element("joint", name="fr3v2_1_finger_joint2_fixed", type="fixed")
        for child in list(finger2):
            fixed.append(child)
        ctrl_root.append(fixed)

    ctrl_path = URDF_DIR / URDF_CTRL_NAME
    _indent(ctrl_root)
    ET.ElementTree(ctrl_root).write(ctrl_path, encoding="utf-8", xml_declaration=True)
    return sim_path, ctrl_path


def _run_urdf_to_mjcf(urdf_path: Path) -> None:
    converter = SCRIPT_DIR / "urdf_to_mjcf.py"
    cmd = [sys.executable, str(converter), str(urdf_path)]
    print("+", " ".join(cmd))
    subprocess.run(cmd, check=True)


def _patch_wrapper_mjcf() -> None:
    """Keep 8 actuators; couple finger_joint2 to finger_joint1 via equality."""
    wrapper = MJCF_DIR / "fr3v2_1_franka_hand_sim.xml"
    config_path = MJCF_DIR / "fr3v2_1_franka_hand_sim_config.yaml"

    import yaml

    with open(config_path, "r", encoding="utf-8") as f:
        config = yaml.safe_load(f)

    finger2 = "fr3v2_1_finger_joint2"
    if finger2 in config.get("actuators", {}):
        del config["actuators"][finger2]
    if finger2 in config.get("joint_properties", {}):
        del config["joint_properties"][finger2]

    with open(config_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(config, f, sort_keys=False, allow_unicode=True)

    tree = ET.parse(wrapper)
    root = tree.getroot()
    if root.find("equality") is None:
        eq = ET.SubElement(root, "equality")
        joint_eq = ET.SubElement(eq, "joint")
        joint_eq.set("joint1", finger2)
        joint_eq.set("joint2", "fr3v2_1_finger_joint1")
        joint_eq.set("polycoef", "0 1 0 0 0")
        _indent(root)
        tree.write(wrapper, encoding="utf-8", xml_declaration=True)

    act_elem = root.find("actuator")
    if act_elem is not None:
        for act in list(act_elem):
            if act.get("joint") == finger2:
                act_elem.remove(act)
        tree.write(wrapper, encoding="utf-8", xml_declaration=True)

    print(f"Patched wrapper: {wrapper}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Prepare FR3v2.1 + Hand MuJoCo assets")
    parser.add_argument(
        "--franka-src",
        type=Path,
        default=Path(
            "/home/qj00462/Documents/Franka/"
            "franka_research3_arm_v2_1_model_package/franka_description"
        ),
        help="Path to vendor franka_description package",
    )
    args = parser.parse_args()

    if not args.franka_src.is_dir():
        raise SystemExit(f"franka_description not found: {args.franka_src}")

    print("Copying meshes...")
    _copy_meshes(args.franka_src)

    print("Writing cleaned URDFs (sim + control)...")
    sim_urdf, ctrl_urdf = _write_urdfs(args.franka_src)
    print(f"  sim   : {sim_urdf}")
    print(f"  ctrl  : {ctrl_urdf}")

    print("Converting sim URDF -> MJCF...")
    _run_urdf_to_mjcf(sim_urdf)

    print("Patching MJCF for 8-DOF control + finger mimic...")
    _patch_wrapper_mjcf()

    print("Done.")
    print(f"  MJCF : {MJCF_DIR / 'fr3v2_1_franka_hand_sim.xml'}")


if __name__ == "__main__":
    main()
