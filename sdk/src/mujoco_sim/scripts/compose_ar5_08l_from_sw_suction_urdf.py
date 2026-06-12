#!/usr/bin/env python3
"""
Compose AR5-5_08L-W4C4A6-ZY2 + suction cup URDF from SolidWorks export.

- Arm geometry from vendor 08L description (link7 mass unchanged at 0.33 kg).
- Suction geometry/joints from hardware SW URDF (suction_cup.SLDASM).
- Fixed mount on flan_link at identity (坐标系1 = flange frame).
- Mass/inertia/COM from hardware measurement (relative to SW 坐标系1 = flan mount).
"""

from __future__ import annotations

import argparse
import re
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

ARM_PREFIX = "AR5-5_08L-W4C4A6-ZY2"
ROBOT_NAME = f"{ARM_PREFIX}_with_suction_cup"
VENDOR_DIR = "AR5-5_08L-W4C4A6-ZY2_description20260511"
OUTPUT_DIR = f"{ARM_PREFIX}_with_suction_cup"

# SW link/joint names in suction_cup.SLDASM.urdf
SW_BASE_LINK = "base_link"
SW_CUP_LINK = "Empty_Link1"
SW_CUP_JOINT = "j1"

# Mass (kg) from 吸盘重量分布-2026-6-9.xlsx — converted to measured total 1.0466 kg.
MASS_BASE_KG = 1.00422732634617
MASS_CUP_KG = 0.0423726736538326

# Base inertia & COM from hardware measurement (吸盘底座).
INERTIAL_BASE = {
    "origin_xyz": (-0.01016387, 0.00166764, 0.04862591),
    "ixx": 0.00381085,
    "ixy": 0.00006744,
    "ixz": -0.00054609,
    "iyy": 0.00432244,
    "iyz": 0.00004154,
    "izz": 0.00158135,
}
# Cup inertia & COM from hardware measurement (吸盘头).
INERTIAL_CUP = {
    "origin_xyz": (0.0, 0.0, 0.00318644),
    "ixx": 0.00002709,
    "ixy": 0.0,
    "ixz": 0.0,
    "iyy": 0.00000463,
    "iyz": 0.0,
    "izz": 0.00002573,
}

SUCTION_LIMITS = (-1.5708, 1.5708, 5.0, 15.708)
SUCTION_JOINT_ORIGIN = (-0.0035, 0.0, 0.2145)

MESH_MAP = {
    SW_BASE_LINK: f"{ARM_PREFIX}_link_suction_cup_base.stl",
    SW_CUP_LINK: f"{ARM_PREFIX}_link_suction_cup.stl",
}


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


def _fix_mesh_paths(root: ET.Element) -> None:
    pattern = re.compile(
        r"package://[^/]+/meshes/(" + re.escape(ARM_PREFIX) + r"_[^\"']+)"
    )
    for mesh in root.iter("mesh"):
        filename = mesh.get("filename", "")
        m = pattern.search(filename)
        if m:
            mesh.set("filename", f"../meshes/{m.group(1)}")


def _add_transmissions(root: ET.Element) -> None:
    joints = []
    for joint in root.findall("joint"):
        name = joint.get("name", "")
        if joint.get("type") != "revolute":
            continue
        if not name.startswith(ARM_PREFIX):
            continue
        if "suction" in name:
            continue
        joints.append(joint)

    for idx, joint in enumerate(joints, start=1):
        jname = joint.get("name")
        tran = ET.Element("transmission", {"name": f"tran{idx}"})
        ET.SubElement(tran, "type").text = "transmission_interface/SimpleTransmission"
        tjoint = ET.SubElement(tran, "joint", {"name": jname})
        ET.SubElement(tjoint, "hardwareInterface").text = "EffortJointInterface"
        actuator = ET.SubElement(tran, "actuator", {"name": f"motor{idx}"})
        ET.SubElement(actuator, "hardwareInterface").text = "EffortJointInterface"
        ET.SubElement(actuator, "mechanicalReduction").text = "1"
        parent = root
        children = list(parent)
        j_index = children.index(joint)
        parent.insert(j_index + 1, tran)


def _ensure_flan_visual(root: ET.Element) -> None:
    flan = root.find(f'.//link[@name="{ARM_PREFIX}_flan_link"]')
    if flan is None or flan.find("visual") is not None:
        return
    visual = ET.SubElement(flan, "visual")
    ET.SubElement(visual, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    geom = ET.SubElement(visual, "geometry")
    ET.SubElement(geom, "sphere", {"radius": "0.005"})
    ET.SubElement(visual, "material", {"name": "grey"})


def _fmt_float(value: float) -> str:
    if value == 0.0:
        return "0"
    return f"{value:.12g}"


def _set_inertial(link: ET.Element, mass_kg: float, inertial: dict[str, object]) -> None:
    iner = link.find("inertial")
    if iner is None:
        iner = ET.SubElement(link, "inertial")

    origin = iner.find("origin")
    if origin is None:
        origin = ET.SubElement(iner, "origin")
    ox, oy, oz = inertial["origin_xyz"]  # type: ignore[misc]
    origin.set("xyz", " ".join(_fmt_float(v) for v in (ox, oy, oz)))
    origin.set("rpy", "0 0 0")

    mass = iner.find("mass")
    if mass is None:
        mass = ET.SubElement(iner, "mass")
    mass.set("value", _fmt_float(mass_kg))

    inertia = iner.find("inertia")
    if inertia is None:
        inertia = ET.SubElement(iner, "inertia")
    for key in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
        inertia.set(key, _fmt_float(float(inertial[key])))  # type: ignore[arg-type]


def _mesh_link(
    link_name: str,
    mesh_rel: str,
    *,
    mass_kg: float,
    inertial: dict[str, object],
    material: str = "grey",
) -> ET.Element:
    link = ET.Element("link", {"name": link_name})
    visual = ET.SubElement(link, "visual")
    geom_v = ET.SubElement(visual, "geometry")
    ET.SubElement(geom_v, "mesh", {"filename": mesh_rel})
    ET.SubElement(visual, "origin", {"rpy": "0 0 0", "xyz": "0 0 0"})
    ET.SubElement(visual, "material", {"name": material})

    _set_inertial(link, mass_kg, inertial)

    collision = ET.SubElement(link, "collision")
    geom_c = ET.SubElement(collision, "geometry")
    ET.SubElement(geom_c, "mesh", {"filename": mesh_rel})
    ET.SubElement(collision, "origin", {"rpy": "0 0 0", "xyz": "0 0 0"})
    return link


def _suction_elements() -> list[ET.Element]:
    base_link = f"{ARM_PREFIX}_link_suction_cup_base"
    base_joint = f"{ARM_PREFIX}_joint_suction_cup_base"
    cup_link = f"{ARM_PREFIX}_link_suction_cup"
    cup_joint = f"{ARM_PREFIX}_joint_suction_cup"
    base_mesh = f"../meshes/{MESH_MAP[SW_BASE_LINK]}"
    cup_mesh = f"../meshes/{MESH_MAP[SW_CUP_LINK]}"

    base = _mesh_link(
        base_link,
        base_mesh,
        mass_kg=MASS_BASE_KG,
        inertial=INERTIAL_BASE,
    )

    fix = ET.Element("joint", {"name": base_joint, "type": "fixed"})
    ET.SubElement(fix, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    ET.SubElement(fix, "parent", {"link": f"{ARM_PREFIX}_flan_link"})
    ET.SubElement(fix, "child", {"link": base_link})

    cup = _mesh_link(
        cup_link,
        cup_mesh,
        mass_kg=MASS_CUP_KG,
        inertial=INERTIAL_CUP,
    )

    lower, upper, effort, velocity = SUCTION_LIMITS
    rev = ET.Element("joint", {"name": cup_joint, "type": "revolute"})
    ET.SubElement(
        rev,
        "origin",
        {
            "xyz": " ".join(_fmt_float(v) for v in SUCTION_JOINT_ORIGIN),
            "rpy": "0 0 0",
        },
    )
    ET.SubElement(rev, "parent", {"link": base_link})
    ET.SubElement(rev, "child", {"link": cup_link})
    ET.SubElement(rev, "axis", {"xyz": "0 1 0"})
    ET.SubElement(
        rev,
        "limit",
        {
            "lower": str(lower),
            "upper": str(upper),
            "effort": str(effort),
            "velocity": str(velocity),
        },
    )
    return [base, fix, cup, rev]


def copy_meshes(sw_meshes: Path, output_meshes: Path, vendor_meshes: Path) -> None:
    output_meshes.mkdir(parents=True, exist_ok=True)
    for src in sorted(vendor_meshes.glob(f"{ARM_PREFIX}_*.stl")):
        shutil.copy2(src, output_meshes / src.name)

    for sw_name, out_name in MESH_MAP.items():
        src = sw_meshes / f"{sw_name}.STL"
        if not src.is_file():
            src = sw_meshes / f"{sw_name}.stl"
        if not src.is_file():
            raise FileNotFoundError(f"SW mesh missing for {sw_name}: {sw_meshes}")
        dst = output_meshes / out_name
        shutil.copy2(src, dst)
        print(f"Copied SW mesh {src.name} → {dst.name}")


def compose_urdf(vendor_urdf: Path, output_urdf: Path) -> None:
    tree = ET.parse(vendor_urdf)
    root = tree.getroot()
    root.set("name", ROBOT_NAME)
    _fix_mesh_paths(root)
    _ensure_flan_visual(root)
    _add_transmissions(root)

    for elem in _suction_elements():
        root.append(elem)

    _indent(root)
    output_urdf.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output_urdf, encoding="UTF-8", xml_declaration=True)
    text = output_urdf.read_text(encoding="utf-8")
    if not text.startswith("<?xml"):
        output_urdf.write_text('<?xml version="1.0" encoding="UTF-8"?>\n' + text, encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--robot-description-root",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "robot_description",
    )
    parser.add_argument(
        "--sw-suction-dir",
        type=Path,
        default=Path("/home/qj00462/Documents/AR5-0.8L/suction_cup.SLDASM"),
        help="SolidWorks-exported suction_cup.SLDASM ROS package directory",
    )
    args = parser.parse_args()

    root = args.robot_description_root
    vendor_urdf = root / VENDOR_DIR / "urdf" / f"{ARM_PREFIX}.urdf"
    vendor_meshes = root / VENDOR_DIR / "meshes"
    sw_meshes = args.sw_suction_dir / "meshes"
    out_dir = root / OUTPUT_DIR
    out_urdf_dir = out_dir / "urdf"
    out_meshes = out_dir / "meshes"

    if not vendor_urdf.is_file():
        raise SystemExit(f"Vendor URDF not found: {vendor_urdf}")
    if not sw_meshes.is_dir():
        raise SystemExit(f"SW meshes dir not found: {sw_meshes}")

    copy_meshes(sw_meshes, out_meshes, vendor_meshes)

    mujoco_urdf = out_urdf_dir / f"{ROBOT_NAME}.urdf"
    compose_urdf(vendor_urdf, mujoco_urdf)
    print(f"Wrote {mujoco_urdf}")
    print(
        f"Mass: base={MASS_BASE_KG:.4f} kg, cup={MASS_CUP_KG:.4f} kg "
        f"(base & cup inertia from hardware measurement)"
    )

    controller_urdf_name = "AR5_08L_suction_cup.urdf"
    for pkg in ("robot_controller", "human_data"):
        dst = Path(__file__).resolve().parents[2] / pkg / "urdf" / controller_urdf_name
        shutil.copy2(mujoco_urdf, dst)
        print(f"Copied to {dst}")


if __name__ == "__main__":
    main()
