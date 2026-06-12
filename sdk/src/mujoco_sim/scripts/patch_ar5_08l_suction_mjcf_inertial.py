#!/usr/bin/env python3
"""
Patch AR5 08L suction cup MJCF inertial bodies.

MuJoCo's URDF importer merges fixed joints (flan_link + suction base) into link7,
which produces a wrong combined COM in the viewer. This script restores:
  link7 (vendor inertial only)
    └─ link_suction_cup_base @ z=0.09 with measured inertial
         └─ link_suction_cup @ revolute offset with measured inertial
"""

from __future__ import annotations

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

ARM_PREFIX = "AR5-5_08L-W4C4A6-ZY2"
ROBOT_DIR = f"{ARM_PREFIX}_with_suction_cup"
BASE_XML = f"{ROBOT_DIR}_base.xml"

LINK7_INERTIAL = {
    "pos": "-0.001562 -0.000366 0.05392",
    "mass": "0.33",
    "fullinertia": "0.00046062 0.00068652 0.00043943 0.00000269 0.00003859 -0.00000095",
}
BASE_INERTIAL = {
    "pos": "-0.01016387 0.00166764 0.04862591",
    "mass": "1.00422732634617",
    "fullinertia": "0.00381085 0.00432244 0.00158135 0.00006744 -0.00054609 0.00004154",
}
CUP_INERTIAL = {
    "pos": "0 0 0.00318644",
    "mass": "0.0423726736538326",
    "fullinertia": "0.00002709 0.00000463 0.00002573 0 0 0",
}
BASE_BODY_POS = "0 0 0.09"
CUP_BODY_POS = "-0.0035 0 0.2145"


def _set_inertial(elem: ET.Element, spec: dict[str, str]) -> None:
    for key in ("pos", "quat", "mass", "diaginertia", "fullinertia"):
        if key in spec:
            elem.set(key, spec[key])
        elif key in elem.attrib:
            del elem.attrib[key]


def patch_mjcf_base(path: Path) -> None:
    tree = ET.parse(path)
    root = tree.getroot()
    link7 = root.find(f".//body[@name='{ARM_PREFIX}_link7']")
    if link7 is None:
        raise RuntimeError(f"{ARM_PREFIX}_link7 body not found in {path}")

    cup = link7.find(f"body[@name='{ARM_PREFIX}_link_suction_cup']")
    if cup is None:
        raise RuntimeError("suction cup body not found under link7")

    # Remove merged base geom on link7.
    for geom in list(link7.findall("geom")):
        if geom.get("mesh") == f"{ARM_PREFIX}_link_suction_cup_base":
            link7.remove(geom)

    # Restore link7-only inertial.
    link7_inertial = link7.find("inertial")
    if link7_inertial is None:
        link7_inertial = ET.Element("inertial")
        joint = link7.find("joint")
        if joint is not None:
            idx = list(link7).index(joint)
            link7.insert(idx, link7_inertial)
        else:
            link7.insert(0, link7_inertial)
    _set_inertial(link7_inertial, LINK7_INERTIAL)

    # Detach cup; it will be reattached under the base body.
    link7.remove(cup)

    base_body = ET.Element("body", {"name": f"{ARM_PREFIX}_link_suction_cup_base", "pos": BASE_BODY_POS})
    base_inertial = ET.SubElement(base_body, "inertial")
    _set_inertial(base_inertial, BASE_INERTIAL)
    ET.SubElement(
        base_body,
        "geom",
        {
            "type": "mesh",
            "rgba": "0.2 0.2 0.2 1",
            "mesh": f"{ARM_PREFIX}_link_suction_cup_base",
        },
    )
    cup.set("pos", CUP_BODY_POS)
    cup_inertial = cup.find("inertial")
    if cup_inertial is None:
        cup_inertial = ET.SubElement(cup, "inertial")
    _set_inertial(cup_inertial, CUP_INERTIAL)
    base_body.append(cup)
    link7.append(base_body)

    tree.write(path, encoding="UTF-8", xml_declaration=True)
    print(f"Patched {path}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mjcf-base",
        type=Path,
        default=Path(__file__).resolve().parents[1]
        / "robot_description"
        / ROBOT_DIR
        / "mjcf"
        / BASE_XML,
    )
    args = parser.parse_args()
    if not args.mjcf_base.is_file():
        raise SystemExit(f"MJCF base not found: {args.mjcf_base}")
    patch_mjcf_base(args.mjcf_base)


if __name__ == "__main__":
    main()
