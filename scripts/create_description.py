#!/usr/bin/env python3
from __future__ import annotations

import argparse
import importlib
import os
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from contextlib import contextmanager
from datetime import datetime
from pathlib import Path


PACKAGE_NAME = "galbot_one_golf_description"
ROBOT_NAME = "galbot_one_golf"
SCRIPT_DIR = Path(__file__).resolve().parent
ISAAC_MARKER_NAME = ".generated_by_galbot_one_golf_description_creator"

EE_OPTIONS = {
    "galbot_gripper": "Galbot gripper",
    "none": "No end effector",
}
ARM_CAMERA_OPTIONS = ("d405", "d415", "none")


def is_package_dir(path: Path) -> bool:
    return (path / "package.xml").is_file() and (path / "xacro").is_dir()


def ament_package_share_dir() -> Path | None:
    try:
        packages = importlib.import_module("ament_index_python.packages")
        return Path(packages.get_package_share_directory(PACKAGE_NAME))
    except Exception:
        return None


def discover_package_dir() -> Path:
    candidates = [
        SCRIPT_DIR.parent,
        SCRIPT_DIR.parent.parent / "share" / PACKAGE_NAME,
    ]
    ament_share = ament_package_share_dir()
    if ament_share is not None:
        candidates.append(ament_share)

    for candidate in candidates:
        if is_package_dir(candidate):
            return candidate

    return SCRIPT_DIR.parent


PACKAGE_DIR = discover_package_dir()


def _import_xacro_module():
    try:
        return importlib.import_module("xacro")
    except ImportError as exc:
        raise RuntimeError(
            "Python xacro package is required. Source a ROS 2 setup file or install xacro."
        ) from exc


def _ros_python_site_packages() -> list[Path]:
    ros_root = Path("/opt/ros")
    if not ros_root.is_dir():
        return []

    distro_names = []
    ros_distro = os.environ.get("ROS_DISTRO")
    if ros_distro:
        distro_names.append(ros_distro)
    distro_names.extend(path.name for path in sorted(ros_root.iterdir()))

    candidates = []
    seen = set()
    for distro_name in distro_names:
        for pattern in ("lib/python*/site-packages", "lib/python*/dist-packages"):
            for path in sorted((ros_root / distro_name).glob(pattern)):
                xacro_init = path / "xacro" / "__init__.py"
                if xacro_init.is_file() and path not in seen:
                    candidates.append(path)
                    seen.add(path)
    return candidates


def require_xacro_module():
    try:
        xacro = _import_xacro_module()
    except RuntimeError:
        xacro = None
    if xacro is not None and hasattr(xacro, "process_file"):
        return xacro

    sys.modules.pop("xacro", None)
    for site_packages in reversed(_ros_python_site_packages()):
        site_packages_str = site_packages.as_posix()
        if site_packages_str not in sys.path:
            sys.path.insert(0, site_packages_str)

    xacro = _import_xacro_module()
    if hasattr(xacro, "process_file"):
        return xacro

    raise RuntimeError(
        "Python xacro package is required. Source a ROS 2 setup file or install xacro."
    )


def in_conda_environment() -> bool:
    return bool(os.environ.get("CONDA_PREFIX") or os.environ.get("CONDA_DEFAULT_ENV"))


def xacro_runtime_hint(exc: Exception) -> RuntimeError:
    message = str(exc)
    if "No module named 'yaml'" in message:
        conda_suffix = ""
        if in_conda_environment():
            conda_name = os.environ.get("CONDA_DEFAULT_ENV") or os.environ.get("CONDA_PREFIX")
            conda_suffix = (
                "\nDetected conda environment: "
                f"{conda_name}"
                "\nThis usually means ROS xacro is being imported from the ROS install,"
                " but PyYAML from the system Python is hidden by conda."
            )

        return RuntimeError(
            "xacro failed because Python could not import 'yaml'."
            f"{conda_suffix}\n"
            "Please run this script outside conda, or explicitly use the system Python.\n"
            "Example:\n"
            "  conda deactivate\n"
            "  source /opt/ros/$ROS_DISTRO/setup.bash\n"
            "  /usr/bin/python3 scripts/create_description.py"
        )

    return RuntimeError(f"xacro failed to render the robot description:\n{message}")


@contextmanager
def source_package_overlay(package_dir: Path):
    """Expose this source package to xacro's $(find ...) lookup without installing it."""
    old_ament_prefix_path = os.environ.get("AMENT_PREFIX_PATH")
    with tempfile.TemporaryDirectory(prefix="galbot_one_golf_ament_") as temp_root:
        prefix = Path(temp_root)
        package_index = prefix / "share" / "ament_index" / "resource_index" / "packages"
        package_index.mkdir(parents=True, exist_ok=True)
        (package_index / PACKAGE_NAME).write_text("", encoding="utf-8")

        share_link = prefix / "share" / PACKAGE_NAME
        share_link.symlink_to(package_dir.resolve(), target_is_directory=True)

        prefixes = [prefix.as_posix()]
        if old_ament_prefix_path:
            prefixes.append(old_ament_prefix_path)
        os.environ["AMENT_PREFIX_PATH"] = os.pathsep.join(prefixes)
        try:
            yield
        finally:
            if old_ament_prefix_path is None:
                os.environ.pop("AMENT_PREFIX_PATH", None)
            else:
                os.environ["AMENT_PREFIX_PATH"] = old_ament_prefix_path


def stable_xacro_banner(urdf_content: str, source_name: str) -> str:
    banner = (
        "<!-- |    This document was autogenerated by xacro from "
        f"{source_name:<30} | -->"
    )
    return "\n".join(
        banner
        if line.startswith("<!-- |    This document was autogenerated by xacro from ")
        else line
        for line in urdf_content.splitlines()
    ) + "\n"


def render_xacro_to_urdf(
    *,
    xacro_path: Path,
    output_path: Path,
    mappings: dict[str, str],
    package_dir: Path = PACKAGE_DIR,
) -> None:
    xacro = require_xacro_module()
    xacro_path = xacro_path.resolve()
    output_path = output_path.resolve()

    output_path.parent.mkdir(parents=True, exist_ok=True)
    temp_output = output_path.with_suffix(output_path.suffix + ".tmp")
    try:
        with source_package_overlay(package_dir):
            doc = xacro.process_file(str(xacro_path), mappings=mappings)
        urdf_content = doc.toprettyxml(indent="  ")
    except Exception as exc:
        raise xacro_runtime_hint(exc) from exc

    urdf_content = stable_xacro_banner(urdf_content, xacro_path.name)
    temp_output.write_text(urdf_content, encoding="utf-8")
    temp_output.replace(output_path)


def rewrite_mesh_paths_relative(urdf_path: Path) -> None:
    """Rewrite package://PACKAGE_NAME/ mesh prefixes to relative paths."""
    urdf_content = urdf_path.read_text(encoding="utf-8")
    urdf_content = urdf_content.replace(f"package://{PACKAGE_NAME}/", "")
    urdf_path.write_text(urdf_content, encoding="utf-8")


def extract_referenced_meshes(urdf_path: Path) -> list[str]:
    """Return the set of mesh paths referenced by the URDF, as relative paths.

    Handles both `package://PACKAGE_NAME/...` and already-relative `meshes/...`
    forms. Only `<mesh filename="...">` references are considered.
    """
    urdf_content = urdf_path.read_text(encoding="utf-8")
    root = ET.fromstring(urdf_content)

    package_prefix = f"package://{PACKAGE_NAME}/"
    referenced: set[str] = set()
    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename")
        if not filename:
            continue
        if filename.startswith(package_prefix):
            filename = filename[len(package_prefix) :]
        referenced.add(filename)
    return sorted(referenced)


def copy_meshes_next_to_urdf(urdf_path: Path) -> Path | None:
    """Copy only the meshes referenced by the URDF next to it.

    This skips unused assets such as collision-generation source files
    (*.xml/*.json) and the `collision/{part}/meshes/` intermediate dirs, since
    they are never referenced by the generated URDF.

    Returns the destination path, or None if the meshes directory is missing.
    """
    meshes_dir = PACKAGE_DIR / "meshes"
    if not meshes_dir.is_dir():
        print(f"Warning: meshes directory not found at {meshes_dir}; skipping mesh copy.")
        return None

    referenced = extract_referenced_meshes(urdf_path)
    dest = urdf_path.parent / "meshes"
    copied = 0
    missing = 0
    for rel_path in referenced:
        src = PACKAGE_DIR / rel_path
        if not src.is_file():
            print(f"Warning: referenced mesh not found, skipping: {src}")
            missing += 1
            continue
        # rel_path is package-relative (e.g. "meshes/visual/..."). Place the
        # file under the destination meshes/ folder preserving its structure.
        rel = Path(rel_path)
        if rel.parts and rel.parts[0] == "meshes":
            target = dest / Path(*rel.parts[1:])
        else:
            target = dest / rel
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, target)
        copied += 1

    print(f"Copied {copied} referenced mesh file(s) to {dest}"
          + (f"; {missing} referenced mesh file(s) missing" if missing else ""))
    return dest


def validate_choice(name: str, value: str, choices: set[str] | tuple[str, ...]) -> str:
    if value not in choices:
        valid = ", ".join(sorted(choices))
        raise ValueError(f"Invalid {name}: {value}. Valid values: {valid}")
    return value


def select_from_options(
    *,
    title: str,
    options: list[tuple[str, str, str]],
    default_key: str = "0",
) -> str:
    while True:
        print(f"\n{title} (press enter to choose default [{default_key}]):")
        for key, label, _value in options:
            print(f"[{key}] {label}")

        choice = input("Enter your choice: ").strip() or default_key
        for key, _label, value in options:
            if choice == key:
                return value

        print("\nInvalid input, please enter a valid number.")


def interactive_config(args: argparse.Namespace) -> argparse.Namespace:
    ee_options = [
        ("0", "Galbot Gripper [Recommended]", "galbot_gripper"),
        ("1", "None", "none"),
    ]
    camera_options = [
        ("0", "D405 [Recommended]", "d405"),
        ("1", "D415", "d415"),
        ("2", "None", "none"),
    ]
    wheel_options = [
        ("0", "Fixed wheels (no wheel joints) [Recommended]", "fixed"),
        ("1", "Continuous wheel joints", "continuous"),
        ("2", "Continuous wheel joints + passive rollers", "continuous_passive"),
    ]

    args.left_ee = select_from_options(
        title="Please select the left arm end effector",
        options=ee_options,
    )
    args.right_ee = select_from_options(
        title="Please select the right arm end effector",
        options=ee_options,
    )
    args.arm_camera = select_from_options(
        title="Please select the wrist camera type",
        options=camera_options,
    )
    wheel_config = select_from_options(
        title="Please select the wheel configuration",
        options=wheel_options,
    )
    args.enable_wheel_joints = wheel_config in ("continuous", "continuous_passive")
    args.enable_passive_wheels = wheel_config == "continuous_passive"

    suggested_name = default_output_name(
        left_ee=args.left_ee,
        right_ee=args.right_ee,
        arm_camera=args.arm_camera,
        enable_wheel_joints=args.enable_wheel_joints,
        enable_passive_wheels=args.enable_passive_wheels,
    )
    suggested_path = args.output_dir / suggested_name
    path_choice = input(
        f"\nPlease enter the output file path "
        f"(press enter to choose default [{suggested_path}]): "
    ).strip()
    if path_choice:
        output_path = Path(path_choice)
        if output_path.suffix != ".urdf":
            output_path = output_path.with_suffix(".urdf")
        args.output = output_path

    print("\nYou have selected:")
    print(f"Left EE: {args.left_ee}")
    print(f"Right EE: {args.right_ee}")
    print(f"Arm camera: {args.arm_camera}")
    print(f"Wheels: {wheel_config}")
    print(f"Output path: {args.output or suggested_path}")

    proceed_choice = input(
        "\nDo you want to proceed with these selections? (y/n) "
        "or press enter to choose default [y]: "
    )
    if proceed_choice.lower() not in ("", "y"):
        raise KeyboardInterrupt("User cancelled.")

    return args


def default_output_name(
    *,
    left_ee: str,
    right_ee: str,
    arm_camera: str,
    enable_wheel_joints: bool = False,
    enable_passive_wheels: bool = False,
) -> str:
    return f"{ROBOT_NAME}.urdf"


def xacro_mappings(args: argparse.Namespace) -> tuple[dict[str, str], str, str]:
    ee = args.ee
    left_ee = args.left_ee or ee
    right_ee = args.right_ee or ee

    valid_ee_types = set(EE_OPTIONS)
    validate_choice("ee", ee, valid_ee_types)
    validate_choice("left-ee", left_ee, valid_ee_types)
    validate_choice("right-ee", right_ee, valid_ee_types)
    validate_choice("arm-camera", args.arm_camera, ARM_CAMERA_OPTIONS)

    mappings = {
        "type": ee,
        "left_ee_type": left_ee,
        "right_ee_type": right_ee,
        "arm_camera": args.arm_camera,
        "enable_wheel_joints": str(args.enable_wheel_joints).lower(),
        "enable_passive_wheels": str(args.enable_passive_wheels).lower(),
        "enable_joint5": str(not args.disable_joint5).lower(),
    }
    return mappings, left_ee, right_ee


def resolve_output_path(args: argparse.Namespace, left_ee: str, right_ee: str) -> Path:
    if args.output:
        return args.output

    output_dir = args.output_dir
    if not output_dir.is_absolute():
        output_dir = PACKAGE_DIR / output_dir

    output_name = args.output_name
    if output_name:
        if not output_name.endswith(".urdf"):
            output_name += ".urdf"
    else:
        output_name = default_output_name(
            left_ee=left_ee,
            right_ee=right_ee,
            arm_camera=args.arm_camera,
            enable_wheel_joints=args.enable_wheel_joints,
            enable_passive_wheels=args.enable_passive_wheels,
        )
    return output_dir / output_name


def process_for_isaac_sim(
    *,
    urdf_path: Path,
    output_dir: Path | None,
    effort_rate: float,
    velocity_rate: float,
) -> Path:
    bundle_dir = output_dir or (Path.home() / "Downloads" / f"isaac_sim_{ROBOT_NAME}")
    bundle_dir = bundle_dir.expanduser().resolve()

    if bundle_dir.exists():
        marker_path = bundle_dir / ISAAC_MARKER_NAME
        if any(bundle_dir.iterdir()) and not marker_path.is_file():
            raise FileExistsError(
                f"Isaac output directory already exists and is not empty: {bundle_dir}. "
                "Choose another --isaac-output-dir or clear it manually."
            )
        shutil.rmtree(bundle_dir)
    bundle_dir.mkdir(parents=True, exist_ok=True)

    bundle_urdf = bundle_dir / urdf_path.name
    shutil.copy2(urdf_path, bundle_urdf)

    meshes_dir = PACKAGE_DIR / "meshes"
    if not meshes_dir.is_dir():
        raise FileNotFoundError(f"Meshes directory not found: {meshes_dir}")
    shutil.copytree(meshes_dir, bundle_dir / "meshes")

    urdf_content = bundle_urdf.read_text(encoding="utf-8")
    urdf_content = urdf_content.replace(f"package://{PACKAGE_NAME}/", "")

    root = ET.fromstring(urdf_content)
    for joint in root.findall(".//joint"):
        limit = joint.find("limit")
        if limit is None:
            continue

        if "effort" in limit.attrib:
            limit.attrib["effort"] = str(float(limit.attrib["effort"]) * effort_rate)
        if "velocity" in limit.attrib:
            limit.attrib["velocity"] = str(float(limit.attrib["velocity"]) * velocity_rate)

    bundle_urdf.write_text(ET.tostring(root, encoding="unicode"), encoding="utf-8")

    version = "unknown"
    package_xml_path = PACKAGE_DIR / "package.xml"
    if package_xml_path.exists():
        try:
            version_element = ET.parse(package_xml_path).getroot().find("version")
            if version_element is not None and version_element.text:
                version = version_element.text.strip()
        except ET.ParseError:
            pass

    git_hash = "unknown"
    try:
        hash_result = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            capture_output=True,
            check=False,
            text=True,
            cwd=PACKAGE_DIR,
        )
        git_hash = hash_result.stdout.strip() or "unknown"
    except Exception:
        pass

    readme_content = (
        "Generated by Galbot One Golf Description Creator\n"
        f"Version: {version}\n"
        f"Name: {ROBOT_NAME}\n"
        f"Hash: {git_hash}\n"
        f"Date: {datetime.now().strftime('%Y-%m-%d')}\n"
        "Note: package:// paths were rewritten for Isaac Sim import, and joint "
        f"limits were scaled by effort={effort_rate}, velocity={velocity_rate}."
    )
    (bundle_dir / "README.md").write_text(readme_content, encoding="utf-8")
    (bundle_dir / ISAAC_MARKER_NAME).write_text(
        "This directory was generated by scripts/create_description.py.\n",
        encoding="utf-8",
    )
    return bundle_dir


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Generate Galbot One Golf URDF descriptions from xacro."
    )
    parser.add_argument(
        "--ee-type",
        "--ee",
        dest="ee",
        default="galbot_gripper",
        choices=sorted(EE_OPTIONS),
        help="Default end effector for both arms.",
    )
    parser.add_argument(
        "--left-ee-type",
        "--left-ee",
        dest="left_ee",
        default="",
        metavar="EE_TYPE",
        help="Left arm end effector. Defaults to --ee-type.",
    )
    parser.add_argument(
        "--right-ee-type",
        "--right-ee",
        dest="right_ee",
        default="",
        metavar="EE_TYPE",
        help="Right arm end effector. Defaults to --ee-type.",
    )
    parser.add_argument(
        "--arm-camera",
        default="d405",
        choices=ARM_CAMERA_OPTIONS,
        help="Wrist camera type.",
    )
    parser.add_argument(
        "--enable-wheel-joints",
        action="store_true",
        help="Generate continuous wheel joints instead of fixed wheel joints.",
    )
    parser.add_argument(
        "--enable-passive-wheels",
        action="store_true",
        help="Generate passive roller links and joints for each omni wheel.",
    )
    parser.add_argument(
        "--disable-joint5",
        action="store_true",
        help="Disable leg_joint5 in the generated model.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="Output URDF path. Overrides --output-dir and generated filename.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("urdf"),
        help="Output directory when --output is not provided.",
    )
    parser.add_argument(
        "--output-name",
        default="",
        metavar="FILENAME",
        help="Output file name within --output-dir. Defaults to a generated name.",
    )
    parser.add_argument(
        "--interactive",
        action="store_true",
        help="Prompt for configuration choices.",
    )
    parser.add_argument(
        "--isaac",
        action="store_true",
        help="Create an Isaac Sim import bundle after generating the URDF.",
    )
    parser.add_argument(
        "--isaac-output-dir",
        type=Path,
        help="Output directory for the Isaac Sim bundle.",
    )
    parser.add_argument(
        "--isaac-effort-rate",
        type=float,
        default=5.0,
        help="Effort multiplier for Isaac Sim bundle joint limits.",
    )
    parser.add_argument(
        "--isaac-velocity-rate",
        type=float,
        default=5.0,
        help="Velocity multiplier for Isaac Sim bundle joint limits.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    if args.interactive or ((argv is None and len(sys.argv) == 1) and sys.stdin.isatty()):
        try:
            args = interactive_config(args)
        except KeyboardInterrupt:
            print("\nExiting program...")
            return 0

    try:
        mappings, left_ee, right_ee = xacro_mappings(args)
    except ValueError as exc:
        parser.error(str(exc))
    xacro_path = PACKAGE_DIR / "xacro" / "robot.xacro"
    output_path = resolve_output_path(args, left_ee, right_ee)

    print("Creating URDF file...")
    print(f"Package: {PACKAGE_DIR}")
    print(f"Xacro: {xacro_path}")
    print(f"Output: {output_path}")
    print(f"Left EE: {left_ee}")
    print(f"Right EE: {right_ee}")

    render_xacro_to_urdf(
        xacro_path=xacro_path,
        output_path=output_path,
        mappings=mappings,
    )

    if args.isaac:
        bundle_dir = process_for_isaac_sim(
            urdf_path=output_path,
            output_dir=args.isaac_output_dir,
            effort_rate=args.isaac_effort_rate,
            velocity_rate=args.isaac_velocity_rate,
        )
        print(f"Isaac Sim bundle: {bundle_dir}")
    else:
        # By default, rewrite package:// mesh paths to relative paths and
        # copy the meshes folder next to the generated URDF so it is portable.
        rewrite_mesh_paths_relative(output_path)
        meshes_dest = copy_meshes_next_to_urdf(output_path)
        if meshes_dest is not None:
            print(f"Meshes copied to: {meshes_dest}")

    print(f"URDF created successfully: {output_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
