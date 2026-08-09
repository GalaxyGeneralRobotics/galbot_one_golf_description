#!/usr/bin/env python3
"""Validate the Galbot gripper TCP transform across every published format."""

from __future__ import annotations

import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
TCP_NAMES = ("left_gripper_tcp_link", "right_gripper_tcp_link")
EXPECTED_POS = (0.13996, 0.0, 0.0)
EXPECTED_RPY = (-math.pi / 2.0, 0.0, 0.0)
EXPECTED_QUAT_WXYZ = (math.sqrt(0.5), -math.sqrt(0.5), 0.0, 0.0)
TOLERANCE = 1e-6


def _values(raw: str, *, count: int, label: str) -> tuple[float, ...]:
    values = tuple(float(value) for value in raw.split())
    if len(values) != count:
        raise ValueError(f"{label} must contain {count} values")
    return values


def _assert_close(
    actual: tuple[float, ...], expected: tuple[float, ...], *, label: str
) -> None:
    if len(actual) != len(expected) or any(
        not math.isclose(left, right, abs_tol=TOLERANCE)
        for left, right in zip(actual, expected, strict=True)
    ):
        raise ValueError(f"{label}: expected {expected}, observed {actual}")


def _assert_quat(actual: tuple[float, ...], *, label: str) -> None:
    norm = math.sqrt(sum(value * value for value in actual))
    if not math.isclose(norm, 1.0, abs_tol=1e-4):
        raise ValueError(f"{label}: quaternion is not normalized: {actual}")
    dot = sum(
        left * right for left, right in zip(actual, EXPECTED_QUAT_WXYZ, strict=True)
    )
    if not math.isclose(abs(dot), 1.0, abs_tol=1e-4):
        raise ValueError(
            f"{label}: expected {EXPECTED_QUAT_WXYZ} up to sign, observed {actual}"
        )


def _validate_mjcf(path: Path) -> None:
    root = ET.parse(path).getroot()
    for name in TCP_NAMES:
        matches = root.findall(f".//body[@name='{name}']")
        if len(matches) != 1:
            raise ValueError(f"{path}: expected one {name}, observed {len(matches)}")
        body = matches[0]
        _assert_close(
            _values(body.attrib.get("pos", ""), count=3, label=f"{path}:{name} pos"),
            EXPECTED_POS,
            label=f"{path}:{name} pos",
        )
        _assert_quat(
            _values(body.attrib.get("quat", ""), count=4, label=f"{path}:{name} quat"),
            label=f"{path}:{name} quat",
        )


def _validate_urdf(path: Path) -> None:
    root = ET.parse(path).getroot()
    for name in TCP_NAMES:
        matches = [
            joint
            for joint in root.findall("joint")
            if (child := joint.find("child")) is not None
            and child.attrib.get("link") == name
        ]
        if len(matches) != 1:
            raise ValueError(
                f"{path}: expected one joint for {name}, observed {len(matches)}"
            )
        origin = matches[0].find("origin")
        if origin is None:
            raise ValueError(f"{path}:{name} has no origin")
        _assert_close(
            _values(origin.attrib.get("xyz", ""), count=3, label=f"{path}:{name} xyz"),
            EXPECTED_POS,
            label=f"{path}:{name} xyz",
        )
        _assert_close(
            _values(origin.attrib.get("rpy", ""), count=3, label=f"{path}:{name} rpy"),
            EXPECTED_RPY,
            label=f"{path}:{name} rpy",
        )


def _validate_xacro(path: Path) -> None:
    root = ET.parse(path).getroot()
    matches = [
        joint
        for joint in root.iter("joint")
        if joint.attrib.get("name") == "${prefix}gripper_tcp_joint"
    ]
    if len(matches) != 1:
        raise ValueError(
            f"{path}: expected one gripper TCP joint, observed {len(matches)}"
        )
    origin = matches[0].find("origin")
    if origin is None:
        raise ValueError(f"{path}: gripper TCP joint has no origin")
    _assert_close(
        _values(origin.attrib.get("xyz", ""), count=3, label=f"{path}:TCP xyz"),
        EXPECTED_POS,
        label=f"{path}:TCP xyz",
    )
    _assert_close(
        _values(origin.attrib.get("rpy", ""), count=3, label=f"{path}:TCP rpy"),
        EXPECTED_RPY,
        label=f"{path}:TCP rpy",
    )


def _usd_block(text: str, name: str, *, path: Path) -> str:
    match = re.search(
        rf'def Xform "{re.escape(name)}"\s*\{{(?P<body>.*?)\n\s*\}}', text, re.S
    )
    if match is None:
        raise ValueError(f"{path}: cannot find USD Xform {name}")
    return match.group("body")


def _validate_usd(path: Path) -> None:
    text = path.read_text(encoding="utf-8")
    for name in TCP_NAMES:
        block = _usd_block(text, name, path=path)
        orient = re.search(r"xformOp:orient\s*=\s*\(([^)]+)\)", block)
        translate = re.search(r"xformOp:translate\s*=\s*\(([^)]+)\)", block)
        if orient is None or translate is None:
            raise ValueError(f"{path}:{name} lacks an authored orient or translate")
        _assert_quat(
            _values(
                orient.group(1).replace(",", " "),
                count=4,
                label=f"{path}:{name} orient",
            ),
            label=f"{path}:{name} orient",
        )
        _assert_close(
            _values(
                translate.group(1).replace(",", " "),
                count=3,
                label=f"{path}:{name} translate",
            ),
            EXPECTED_POS,
            label=f"{path}:{name} translate",
        )


def main() -> int:
    try:
        _validate_xacro(ROOT / "xacro/components/galbot_gripper.xacro")
        for path in (
            ROOT / "urdf/galbot_one_golf.urdf",
            ROOT / "urdf/galbot_one_golf_fixed_base.urdf",
        ):
            _validate_urdf(path)
        for path in (
            ROOT / "mjcf/galbot_one_golf.xml",
            ROOT / "mjcf/galbot_one_golf_fixed_base.xml",
            ROOT / "mjcf/galbot_one_golf_planar_base.xml",
        ):
            _validate_mjcf(path)
        _validate_usd(ROOT / "usd/payloads/base.usda")
    except (ET.ParseError, OSError, ValueError) as error:
        print(f"TCP frame validation failed: {error}", file=sys.stderr)
        return 1
    print("TCP frames agree across Xacro, URDF, MJCF, and USD.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
