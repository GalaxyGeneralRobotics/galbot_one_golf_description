#!/usr/bin/env python3
"""Check finger collision regions and materials (usd-core, numpy, scipy)."""

import struct
from pathlib import Path

import numpy as np
from scipy.spatial import ConvexHull
from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade


ROOT = Path(__file__).resolve().parent.parent


def stl_points(path):
    data = path.read_bytes()
    count = struct.unpack_from("<I", data, 80)[0]
    return [
        struct.unpack_from("<3f", data, 84 + 50 * i + 12 + 12 * j)
        for i in range(count)
        for j in range(3)
    ]


def rounded_points(points):
    return {tuple(round(c, 7) for c in point) for point in points}


def validate(stage, root):
    for arm in ("left", "right"):
        for side in ("l", "r"):
            link = stage.GetPrimAtPath(f"{root}/{arm}_gripper_{side}_finger_link")
            colliders = [
                prim for prim in Usd.PrimRange(link, Usd.TraverseInstanceProxies())
                if prim.HasAPI(UsdPhysics.CollisionAPI)
                and UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()
            ]
            assert len(colliders) == 5, f"{link.GetPath()}: expected three shells and two pads"
            suffix = "_right" if side == "r" else ""
            sign = -1 if side == "r" else 1
            regions = {}
            for prim in colliders:
                assert "/collisions/" in str(prim.GetPath()), prim.GetPath()
                assert prim.IsA(UsdGeom.Mesh), prim.GetPath()
                assert UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get() == "convexHull"
                transform = UsdGeom.XformCache().ComputeRelativeTransform(prim, link)[0]
                assert transform.GetDeterminant() > 0, prim.GetPath()
                name = str(prim.GetPath()).split("/collisions/")[1].split("/")[0]
                regions[name] = np.array([
                    transform.Transform(Gf.Vec3d(*point))
                    for point in UsdGeom.Mesh(prim).GetPointsAttr().Get()
                ])
                material, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial("physics")
                assert material and material.GetPrim().HasAPI(UsdPhysics.MaterialAPI), prim.GetPath()
                physics = UsdPhysics.MaterialAPI(material)
                static, dynamic = (1.5, 1.5) if name.endswith("_pad") else (0.3, 0.2)
                assert abs(physics.GetStaticFrictionAttr().Get() - static) < 1e-6, material.GetPath()
                assert abs(physics.GetDynamicFrictionAttr().Get() - dynamic) < 1e-6, material.GetPath()
                assert physics.GetRestitutionAttr().Get() == 0, material.GetPath()
            for i in (1, 2, 3):
                name = f"link_3{suffix}_collision_{i:02}"
                source = ROOT / "meshes/collision/end_effectors/galbot_gripper" / f"link3{suffix}" / f"{name}.stl"
                points = np.array(stl_points(source))
                shell = regions[name]
                if i == 3:
                    assert rounded_points(shell) == rounded_points(points), name
                    continue
                pad = regions[name + "_pad"]
                surface = max(points[:, 1] * sign)
                cut = surface - 0.001
                # Disjoint interiors, a 1 mm pad, and no outward growth.
                assert max(shell[:, 1] * sign) <= cut + 1e-8, name
                assert abs(min(pad[:, 1] * sign) - cut) < 1e-8, name
                assert abs(max(pad[:, 1] * sign) - surface) < 1e-8, name
                original = ConvexHull(points)
                combined = np.vstack((shell, pad))
                assert np.max(combined @ original.equations[:, :3].T + original.equations[:, 3]) < 1e-8, name
                volume = ConvexHull(shell).volume + ConvexHull(pad).volume
                assert abs(volume / original.volume - 1) < 1e-5, name
                # The original inner face is fully covered, not reduced to a narrow strip.
                inner = points[np.isclose(points[:, 1] * sign, surface, atol=1e-8)]
                pad_hull = ConvexHull(pad)
                assert np.max(inner @ pad_hull.equations[:, :3].T + pad_hull.equations[:, 3]) < 1e-8, name

    print(f"PASS: 20 gripper colliders, unchanged hull envelope, and separate pad/shell materials at {root}")


if __name__ == "__main__":
    asset = ROOT / "usd/galbot_one_golf.usda"
    validate(Usd.Stage.Open(str(asset)), "/galbot_one_golf")
    # Material targets must also resolve when a consumer references the robot elsewhere.
    stage = Usd.Stage.CreateInMemory()
    stage.DefinePrim("/World/Robot").GetReferences().AddReference(str(asset))
    validate(stage, "/World/Robot")
