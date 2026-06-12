#!/usr/bin/env python3
"""
Compose AR5-5_08L-W4C4A6-ZY2 + suction cup URDF (legacy STL rebase path).

Superseded by compose_ar5_08l_from_sw_suction_urdf.py (hardware SW export).
"""

from __future__ import annotations

import argparse
import re
import shutil
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

ARM_PREFIX = "AR5-5_08L-W4C4A6-ZY2"
ROBOT_NAME = f"{ARM_PREFIX}_with_suction_cup"
VENDOR_DIR = "AR5-5_08L-W4C4A6-ZY2_description20260511"
OUTPUT_DIR = f"{ARM_PREFIX}_with_suction_cup"
SUCTION_SOURCE_PREFIX = "AR5-5_07L-W4C4A2"
SUCTION_07L_MESH = (
    Path(__file__).resolve().parents[1]
    / "robot_description"
    / "AR5-5_07L-W4C4A2_with_suction_cup"
    / "meshes"
    / f"{SUCTION_SOURCE_PREFIX}_link_suction_cup.stl"
)
DEFAULT_BASE_STL = (
    Path(__file__).resolve().parents[1]
    / "robot_description"
    / OUTPUT_DIR
    / "meshes"
    / f"{ARM_PREFIX}_link_suction_cup_base.stl"
)

# Mount base on link7 at flange pose (equivalent to flan_link + fixed base).
# 07L used (-0.0035, 0, 0.3115) from link7 for the revolute axis.
BASE_MOUNT_XYZ = (-0.0035, 0.0, 0.09)
BASE_MOUNT_RPY = (0.0, 0.0, 0.0)
BASE_MOUNT_PARENT = f"{ARM_PREFIX}_link7"
SUCTION_MOUNT_RPY = (0.0, 0.0, 0.0)

# 07L rotating cup mesh (meters); z_min used to butt joint against base top.
CUP_MESH_Z_MIN_M = -0.012921923771500587

# 07L link7(1.156) - bare(0.33) - cup(0.035) ≈ 0.791 kg housing
BASE_LINK_MASS_KG = 0.791
BASE_MESH_SCALE = "0.001 0.001 0.001"


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
    """Insert SimpleTransmission blocks after each arm revolute joint (1..7)."""
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
    """Invisible links break many URDF viewers; add a tiny visual on flan_link."""
    flan = root.find(f'.//link[@name="{ARM_PREFIX}_flan_link"]')
    if flan is None or flan.find("visual") is not None:
        return
    visual = ET.SubElement(flan, "visual")
    ET.SubElement(visual, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    geom = ET.SubElement(visual, "geometry")
    ET.SubElement(geom, "sphere", {"radius": "0.005"})
    ET.SubElement(visual, "material", {"name": "grey"})


def _detect_mount_plane_mm(
    triangles: list[tuple[tuple[float, float, float], tuple, bytes]],
) -> tuple[float, float, float]:
    """Find the flange mounting plane (largest XY cross-section near the bottom)."""
    verts = [v for _, tri, _ in triangles for v in tri]
    z_min = min(v[2] for v in verts)
    best_z = z_min
    best_score = 0.0
    ox = (min(v[0] for v in verts) + max(v[0] for v in verts)) / 2.0
    oy = (min(v[1] for v in verts) + max(v[1] for v in verts)) / 2.0

    z_probe = z_min
    while z_probe < z_min + 80.0:
        slab = [v for v in verts if abs(v[2] - z_probe) < 2.0]
        if len(slab) >= 30:
            xs = [v[0] for v in slab]
            ys = [v[1] for v in slab]
            score = (max(xs) - min(xs)) * (max(ys) - min(ys))
            if score > best_score:
                best_score = score
                best_z = z_probe
                ox = (max(xs) + min(xs)) / 2.0
                oy = (max(ys) + min(ys)) / 2.0
        z_probe += 0.5
    return ox, oy, best_z


def rebase_binary_stl(src: Path, dst: Path) -> tuple[list[float], list[float], float]:
    """
    Rebase binary STL so the detected flange mount plane is at the origin.

    The bbox bottom is often a small feature, not the flange face. Aligning the
    mount plane (largest XY slice) fixes the housing floating above the wrist.

    Returns (mins, maxs) in mm after rebase, and revolute joint Z in meters.
    """
    data = src.read_bytes()
    if data[:5] == b"solid":
        raise ValueError(f"ASCII STL not supported: {src}")

    tri_count = struct.unpack("<I", data[80:84])[0]
    triangles: list[tuple[tuple[float, float, float], tuple]] = []
    off = 84
    for _ in range(tri_count):
        normal = struct.unpack("<fff", data[off : off + 12])
        off += 12
        verts = tuple(
            struct.unpack("<fff", data[off + i * 12 : off + i * 12 + 12]) for i in range(3)
        )
        off += 36
        attr = data[off : off + 2]
        off += 2
        triangles.append((normal, verts, attr))

    ox, oy, oz = _detect_mount_plane_mm(triangles)

    mins = [1e18, 1e18, 1e18]
    maxs = [-1e18, -1e18, -1e18]
    body = bytearray()
    for normal, verts, attr in triangles:
        body.extend(struct.pack("<fff", *normal))
        for x, y, z in verts:
            rx, ry, rz = x - ox, y - oy, z - oz
            mins[0] = min(mins[0], rx)
            mins[1] = min(mins[1], ry)
            mins[2] = min(mins[2], rz)
            maxs[0] = max(maxs[0], rx)
            maxs[1] = max(maxs[1], ry)
            maxs[2] = max(maxs[2], rz)
            body.extend(struct.pack("<fff", rx, ry, rz))
        body.extend(attr)

    out = bytearray(80)
    out.extend(struct.pack("<I", len(triangles)))
    out.extend(body)
    dst.parent.mkdir(parents=True, exist_ok=True)
    dst.write_bytes(out)

    base_top_m = maxs[2] * 0.001
    suction_mount_z_m = base_top_m - CUP_MESH_Z_MIN_M
    return mins, maxs, suction_mount_z_m


def _mesh_link(
    link_name: str,
    mesh_rel: str,
    *,
    scale: str | None,
    mass: float,
    com_xyz: tuple[float, float, float],
    material: str = "grey",
) -> ET.Element:
    link = ET.Element("link", {"name": link_name})
    visual = ET.SubElement(link, "visual")
    geom_v = ET.SubElement(visual, "geometry")
    attrs = {"filename": mesh_rel}
    if scale:
        attrs["scale"] = scale
    ET.SubElement(geom_v, "mesh", attrs)
    ET.SubElement(visual, "origin", {"rpy": "0 0 0", "xyz": "0 0 0"})
    ET.SubElement(visual, "material", {"name": material})

    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "mass", {"value": f"{mass:.6f}".rstrip("0").rstrip(".")})
    ext = max(com_xyz)
    i = 0.01 * mass * ext * ext
    ET.SubElement(
        inertial,
        "inertia",
        {"ixx": f"{i:.8f}", "ixy": "0", "ixz": "0", "iyy": f"{i:.8f}", "iyz": "0", "izz": f"{i:.8f}"},
    )
    ET.SubElement(
        inertial,
        "origin",
        {"rpy": "0 0 0", "xyz": " ".join(f"{v:.6f}" for v in com_xyz)},
    )

    collision = ET.SubElement(link, "collision")
    geom_c = ET.SubElement(collision, "geometry")
    cattrs = {"filename": mesh_rel}
    if scale:
        cattrs["scale"] = scale
    ET.SubElement(geom_c, "mesh", cattrs)
    ET.SubElement(collision, "origin", {"rpy": "0 0 0", "xyz": "0 0 0"})
    return link


def _suction_elements(
    base_com_m: tuple[float, float, float],
    suction_mount_z_m: float,
    suction_limits: tuple[float, float, float, float],
) -> list[ET.Element]:
    base_link = f"{ARM_PREFIX}_link_suction_cup_base"
    base_joint = f"{ARM_PREFIX}_joint_suction_cup_base"
    cup_link = f"{ARM_PREFIX}_link_suction_cup"
    cup_joint = f"{ARM_PREFIX}_joint_suction_cup"
    base_mesh = f"../meshes/{base_link}.stl"
    cup_mesh = f"../meshes/{cup_link}.stl"

    base = _mesh_link(
        base_link,
        base_mesh,
        scale=BASE_MESH_SCALE,
        mass=BASE_LINK_MASS_KG,
        com_xyz=base_com_m,
        material="grey",
    )

    fix = ET.Element("joint", {"name": base_joint, "type": "fixed"})
    ET.SubElement(
        fix,
        "origin",
        {
            "xyz": " ".join(str(v) for v in BASE_MOUNT_XYZ),
            "rpy": " ".join(str(v) for v in BASE_MOUNT_RPY),
        },
    )
    ET.SubElement(fix, "parent", {"link": BASE_MOUNT_PARENT})
    ET.SubElement(fix, "child", {"link": base_link})

    cup = _mesh_link(
        cup_link,
        cup_mesh,
        scale=None,
        mass=0.035,
        com_xyz=(0.0, 0.0, 0.003186),
        material="grey",
    )
    # Override cup inertia with calibrated 07L values
    cup_inertial = cup.find("inertial")
    assert cup_inertial is not None
    cup_inertial.find("inertia").attrib.update(  # type: ignore[union-attr]
        {
            "ixx": "0.000000022282",
            "ixy": "0.0",
            "ixz": "0.0",
            "iyy": "0.000000003806",
            "iyz": "0.0",
            "izz": "0.000000021168",
        }
    )

    lower, upper, effort, velocity = suction_limits
    rev = ET.Element("joint", {"name": cup_joint, "type": "revolute"})
    ET.SubElement(
        rev,
        "origin",
        {
            "xyz": f"0.0 0.0 {suction_mount_z_m:.6f}",
            "rpy": " ".join(str(v) for v in SUCTION_MOUNT_RPY),
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


def compose_urdf(
    vendor_urdf: Path,
    output_urdf: Path,
    *,
    add_transmission: bool,
    base_com_m: tuple[float, float, float],
    suction_mount_z_m: float,
    suction_limits: tuple[float, float, float, float],
) -> None:
    tree = ET.parse(vendor_urdf)
    root = tree.getroot()
    root.set("name", ROBOT_NAME)
    _fix_mesh_paths(root)
    _ensure_flan_visual(root)

    if add_transmission:
        _add_transmissions(root)

    for elem in _suction_elements(base_com_m, suction_mount_z_m, suction_limits):
        root.append(elem)

    _indent(root)
    output_urdf.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output_urdf, encoding="UTF-8", xml_declaration=True)
    text = output_urdf.read_text(encoding="utf-8")
    if not text.startswith("<?xml"):
        output_urdf.write_text('<?xml version="1.0" encoding="UTF-8"?>\n' + text, encoding="utf-8")


def copy_meshes(
    vendor_meshes: Path,
    output_meshes: Path,
    suction_base_src: Path,
) -> tuple[tuple[float, float, float], float]:
    output_meshes.mkdir(parents=True, exist_ok=True)
    for src in sorted(vendor_meshes.glob(f"{ARM_PREFIX}_*.stl")):
        shutil.copy2(src, output_meshes / src.name)

    dst_base = output_meshes / f"{ARM_PREFIX}_link_suction_cup_base.stl"
    mins, maxs, suction_mount_z_m = rebase_binary_stl(suction_base_src, dst_base)
    scale = 0.001
    com_m = (
        (mins[0] + maxs[0]) / 2.0 * scale,
        (mins[1] + maxs[1]) / 2.0 * scale,
        (mins[2] + maxs[2]) / 2.0 * scale,
    )
    print(
        f"Rebased suction base mesh → {dst_base.name} "
        f"(mm ext: {[maxs[i]-mins[i] for i in range(3)]}, com_m={com_m}, "
        f"revolute_z={suction_mount_z_m:.4f}m)"
    )

    dst_cup = output_meshes / f"{ARM_PREFIX}_link_suction_cup.stl"
    shutil.copy2(SUCTION_07L_MESH, dst_cup)
    return com_m, suction_mount_z_m


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--robot-description-root",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "robot_description",
    )
    parser.add_argument(
        "--suction-base-stl",
        type=Path,
        default=Path("/home/qj00462/Downloads/吸盘末端-压缩.STL"),
        help=(
            "Fixed suction housing STL (mm, rebased to mount face at origin). "
            f"Output: {ARM_PREFIX}_link_suction_cup_base.stl"
        ),
    )
    args = parser.parse_args()

    root = args.robot_description_root
    vendor_urdf = root / VENDOR_DIR / "urdf" / f"{ARM_PREFIX}.urdf"
    vendor_meshes = root / VENDOR_DIR / "meshes"
    out_dir = root / OUTPUT_DIR
    out_urdf_dir = out_dir / "urdf"
    out_meshes = out_dir / "meshes"

    if not vendor_urdf.is_file():
        raise SystemExit(f"Vendor URDF not found: {vendor_urdf}")
    if not args.suction_base_stl.is_file():
        raise SystemExit(f"Suction base STL not found: {args.suction_base_stl}")
    if not SUCTION_07L_MESH.is_file():
        raise SystemExit(f"Suction cup mesh not found: {SUCTION_07L_MESH}")

    base_com_m, suction_mount_z_m = copy_meshes(
        vendor_meshes, out_meshes, args.suction_base_stl
    )

    mujoco_urdf = out_urdf_dir / f"{ROBOT_NAME}.urdf"
    compose_urdf(
        vendor_urdf,
        mujoco_urdf,
        add_transmission=True,
        base_com_m=base_com_m,
        suction_mount_z_m=suction_mount_z_m,
        suction_limits=(-1.5708, 1.5708, 5.0, 15.708),
    )
    print(f"Wrote {mujoco_urdf}")

    controller_urdf_name = "AR5_08L_suction_cup.urdf"
    for pkg in ("robot_controller", "human_data"):
        dst = Path(__file__).resolve().parents[2] / pkg / "urdf" / controller_urdf_name
        shutil.copy2(mujoco_urdf, dst)
        print(f"Copied to {dst}")


if __name__ == "__main__":
    main()
