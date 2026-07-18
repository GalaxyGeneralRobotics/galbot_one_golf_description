#!/usr/bin/env python3
"""Finalize generated G1 MJCF geometry that URDF cannot express directly."""

from __future__ import annotations

import argparse
import math
import os
import re
import stat
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path


ROLLER_CENTER_RADIUS = 0.08
ROLLER_RADIUS = 0.025
ROLLER_CAPSULE_HALF_LENGTH = 0.0075
ROLLER_URDF_CYLINDER_HALF_LENGTH = ROLLER_RADIUS + ROLLER_CAPSULE_HALF_LENGTH
ROLLER_COUNT = 40

MOBILE_ROLLER_PATTERN = re.compile(
    r"wheel_[1-4]_passive_[0-9]_collision"
)
RIGID_WHEEL_ROLLER_PATTERN = re.compile(r"wheel_[1-4]_collision_[0-9]")


def _floats(value: str) -> tuple[float, ...]:
    return tuple(float(part) for part in value.split())


def _require_close(actual: float, expected: float, context: str) -> None:
    if not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-9):
        raise ValueError(f"{context}: expected {expected}, got {actual}")


def _require_center(position: tuple[float, ...], context: str) -> float:
    if len(position) != 3:
        raise ValueError(f"{context}: expected a three-dimensional position")
    _require_close(position[0], 0.0, f"{context} x position")
    center_radius = math.hypot(position[1], position[2])
    _require_close(center_radius, ROLLER_CENTER_RADIUS, f"{context} center radius")
    return math.atan2(position[2], position[1])


def _normalize_capsule(geom: ET.Element, context: str) -> bool:
    geom_type = geom.get("type")
    size = _floats(geom.get("size", ""))
    if geom_type == "capsule":
        if len(size) != 2:
            raise ValueError(f"{context}: capsule size must contain two values")
        _require_close(size[0], ROLLER_RADIUS, f"{context} capsule radius")
        _require_close(
            size[1],
            ROLLER_CAPSULE_HALF_LENGTH,
            f"{context} capsule half-length",
        )
        return False

    if geom_type != "cylinder":
        raise ValueError(
            f"{context}: expected generated cylinder or finalized capsule, "
            f"got {geom_type!r}"
        )
    if len(size) != 2:
        raise ValueError(f"{context}: cylinder size must contain two values")
    _require_close(size[0], ROLLER_RADIUS, f"{context} cylinder radius")
    _require_close(
        size[1],
        ROLLER_URDF_CYLINDER_HALF_LENGTH,
        f"{context} cylinder half-length",
    )
    geom.set("type", "capsule")
    geom.set("size", f"{ROLLER_RADIUS} {ROLLER_CAPSULE_HALF_LENGTH}")
    return True


def _finalize_mobile_rollers(
    geoms: list[ET.Element], parent_by_child: dict[ET.Element, ET.Element]
) -> int:
    changed = 0
    for geom in geoms:
        name = geom.get("name", "")
        body = parent_by_child.get(geom)
        if body is None or body.tag != "body":
            raise ValueError(f"{name}: passive roller geom must be a direct body child")
        _require_center(_floats(body.get("pos", "")), name)
        joint = body.find("joint")
        if joint is None or _floats(joint.get("axis", "")) != (0.0, 0.0, 1.0):
            raise ValueError(f"{name}: passive roller joint must use local Z axis")
        changed += _normalize_capsule(geom, name)
    return changed


def _finalize_rigid_wheel_rollers(geoms: list[ET.Element]) -> int:
    changed = 0
    for geom in geoms:
        name = geom.get("name", "")
        angle = _require_center(_floats(geom.get("pos", "")), name)
        changed += _normalize_capsule(geom, name)
        for orientation in ("quat", "axisangle", "xyaxes", "zaxis"):
            geom.attrib.pop(orientation, None)
        expected_euler = f"{angle:.17g} 0 0"
        if geom.get("euler") != expected_euler:
            geom.set("euler", expected_euler)
            changed += 1
    return changed


def finalize_mjcf(path: Path, *, check: bool = False) -> int:
    """Normalize one generated MJCF and return its number of edits.

    Args:
        path: MJCF XML file generated from a G1 URDF.
        check: Validate that no finalization edits are needed.

    Returns:
        Number of geometry or orientation edits made.

    Raises:
        ValueError: If the generated geometry does not match the G1 roller contract.
    """
    tree = ET.parse(path)
    root = tree.getroot()
    parent_by_child = {
        child: parent for parent in root.iter() for child in parent
    }
    named_geoms = [geom for geom in root.iter("geom") if geom.get("name")]
    mobile_geoms = [
        geom
        for geom in named_geoms
        if MOBILE_ROLLER_PATTERN.fullmatch(geom.get("name", ""))
    ]
    rigid_geoms = [
        geom
        for geom in named_geoms
        if RIGID_WHEEL_ROLLER_PATTERN.fullmatch(geom.get("name", ""))
    ]
    if bool(mobile_geoms) == bool(rigid_geoms):
        raise ValueError(
            f"{path}: expected exactly one G1 roller geometry family"
        )
    geoms = mobile_geoms or rigid_geoms
    if len(geoms) != ROLLER_COUNT:
        raise ValueError(
            f"{path}: expected {ROLLER_COUNT} roller geoms, got {len(geoms)}"
        )

    if mobile_geoms:
        changed = _finalize_mobile_rollers(mobile_geoms, parent_by_child)
    else:
        changed = _finalize_rigid_wheel_rollers(rigid_geoms)

    if check and changed:
        raise ValueError(f"{path}: requires {changed} MJCF finalization edits")
    if changed:
        source_stat = path.stat()
        ET.indent(tree, space="  ")
        with tempfile.NamedTemporaryFile(
            mode="wb", dir=path.parent, delete=False
        ) as temporary_file:
            temporary_path = Path(temporary_file.name)
            tree.write(temporary_file, encoding="utf-8", xml_declaration=True)
        os.chmod(temporary_path, stat.S_IMODE(source_stat.st_mode))
        temporary_stat = temporary_path.stat()
        if (temporary_stat.st_uid, temporary_stat.st_gid) != (
            source_stat.st_uid,
            source_stat.st_gid,
        ):
            os.chown(temporary_path, source_stat.st_uid, source_stat.st_gid)
        os.replace(temporary_path, path)
    return changed


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mjcf", nargs="+", type=Path, help="MJCF XML path")
    parser.add_argument(
        "--check",
        action="store_true",
        help="validate finalized geometry without writing files",
    )
    args = parser.parse_args()

    for path in args.mjcf:
        edits = finalize_mjcf(path, check=args.check)
        print(f"{path}: {edits} edits")


if __name__ == "__main__":
    main()
