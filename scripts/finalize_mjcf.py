#!/usr/bin/env python3
"""Finalize generated G1 MJCF geometry that URDF cannot express directly."""

from __future__ import annotations

import argparse
import hashlib
import json
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
GRIPPER_FINGER_COUNT = 4

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
GRIPPER_CONTACT_PROFILE_PATH = PACKAGE_ROOT / "config/mjcf/gripper_contact.json"

MOBILE_ROLLER_PATTERN = re.compile(r"wheel_[1-4]_passive_[0-9]_collision")
RIGID_WHEEL_ROLLER_PATTERN = re.compile(r"wheel_[1-4]_collision_[0-9]")
GRIPPER_FINGER_BODY_PATTERN = re.compile(
    r"(?P<robot_side>left|right)_gripper_(?P<finger_side>[lr])_finger_link"
)


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


def _format_vector(values: list[float]) -> str:
    return " ".join(str(float(value)) for value in values)


def _load_gripper_contact_profile() -> dict:
    profile = json.loads(GRIPPER_CONTACT_PROFILE_PATH.read_text(encoding="utf-8"))
    if profile.get("version") != 2:
        raise ValueError("G1 gripper contact profile must use version 2")

    source = profile.get("source", {})
    source_mesh = PACKAGE_ROOT / source.get("mesh", "")
    if not source_mesh.is_file():
        raise ValueError(f"G1 gripper contact source mesh is missing: {source_mesh}")
    source_digest = hashlib.sha256(source_mesh.read_bytes()).hexdigest()
    if source_digest != source.get("sha256"):
        raise ValueError(
            "G1 gripper contact profile is stale for its source visual mesh"
        )

    boxes = profile.get("fit", {}).get("boxes", [])
    expected_names = {
        "pad_proximal_collision",
        "pad_distal_collision",
        "tip_collision",
        *(f"side_shell_{index:02d}_collision" for index in range(8)),
    }
    if (
        len(boxes) != len(expected_names)
        or {box.get("name") for box in boxes} != expected_names
    ):
        raise ValueError("G1 gripper contact profile has incomplete fitted boxes")
    return profile


def _fitted_gripper_geom(
    *, body_name: str, finger_side: str, box: dict, contact: dict
) -> ET.Element:
    position = [float(value) for value in box["pos"]]
    if finger_side == "r":
        position[1] *= -1.0
    attributes = {
        "name": f"{body_name.removesuffix('_link')}_{box['name']}",
        "type": str(box["type"]),
        "pos": _format_vector(position),
        "size": _format_vector(box["size"]),
        "friction": _format_vector(box["friction"]),
        "class": "collision",
        "condim": str(int(contact["condim"])),
        "priority": str(int(contact["priority"])),
        "solref": _format_vector(contact["solref"]),
    }
    if "quat" in box:
        attributes["quat"] = _format_vector(box["quat"])
    return ET.Element("geom", attributes)


def _finalize_gripper_contacts(root: ET.Element) -> int:
    profile = _load_gripper_contact_profile()
    boxes = profile["fit"]["boxes"]
    contact = profile["contact"]
    finger_bodies = [
        body
        for body in root.iter("body")
        if GRIPPER_FINGER_BODY_PATTERN.fullmatch(body.get("name", ""))
    ]
    if len(finger_bodies) != GRIPPER_FINGER_COUNT:
        raise ValueError(
            f"Expected {GRIPPER_FINGER_COUNT} G1 finger bodies, "
            f"got {len(finger_bodies)}"
        )

    changed = 0
    for body in finger_bodies:
        body_name = body.get("name", "")
        match = GRIPPER_FINGER_BODY_PATTERN.fullmatch(body_name)
        assert match is not None
        fitted_names = {
            f"{body_name.removesuffix('_link')}_{box['name']}" for box in boxes
        }
        direct_geoms = body.findall("geom")
        current_fitted = {
            geom.get("name", "")
            for geom in direct_geoms
            if geom.get("name") in fitted_names
        }
        if current_fitted == fitted_names:
            expected_by_name = {
                expected.get("name", ""): expected
                for box in boxes
                for expected in (
                    _fitted_gripper_geom(
                        body_name=body_name,
                        finger_side=match.group("finger_side"),
                        box=box,
                        contact=contact,
                    ),
                )
            }
            for geom in direct_geoms:
                expected = expected_by_name.get(geom.get("name", ""))
                if expected is not None and geom.attrib != expected.attrib:
                    geom.attrib.clear()
                    geom.attrib.update(expected.attrib)
                    changed += 1
            continue
        if current_fitted:
            raise ValueError(f"{body_name}: partially finalized gripper contacts")

        generated_meshes = [
            geom
            for geom in direct_geoms
            if re.fullmatch(
                rf"{re.escape(body_name)}_collision_[0-2]", geom.get("name", "")
            )
        ]
        if len(generated_meshes) != 3:
            raise ValueError(
                f"{body_name}: expected three generated finger collision meshes, "
                f"got {len(generated_meshes)}"
            )
        for geom in generated_meshes:
            body.remove(geom)
        for box in boxes:
            body.append(
                _fitted_gripper_geom(
                    body_name=body_name,
                    finger_side=match.group("finger_side"),
                    box=box,
                    contact=contact,
                )
            )
        changed += len(generated_meshes) + len(boxes)
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
    parent_by_child = {child: parent for parent in root.iter() for child in parent}
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
        raise ValueError(f"{path}: expected exactly one G1 roller geometry family")
    geoms = mobile_geoms or rigid_geoms
    if len(geoms) != ROLLER_COUNT:
        raise ValueError(
            f"{path}: expected {ROLLER_COUNT} roller geoms, got {len(geoms)}"
        )

    if mobile_geoms:
        changed = _finalize_mobile_rollers(mobile_geoms, parent_by_child)
    else:
        changed = _finalize_rigid_wheel_rollers(rigid_geoms)
    changed += _finalize_gripper_contacts(root)

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
