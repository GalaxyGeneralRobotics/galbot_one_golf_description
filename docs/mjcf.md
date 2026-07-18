# MJCF Generation

Run commands from the package root.

The generated `mjcf/` directory should contain only the selected MJCF XML and
shared mesh assets:

- `galbot_one_golf.xml`
- `galbot_one_golf_fixed_base.xml` when generating the fixed-base variant
- `meshes/`

Joint parameters, actuators, joint sensors, and MJCF-only extra joints are
configured through `config/mjcf/joint_data*.json`. Shared MJCF appendices such
as `option` and `contact` are configured through `config/mjcf/appendix.xml`.
The old `default*.json`, `actuator*.json`, and `metadata*.json` inputs are no
longer used for the standard generation commands.

## Required Roller Finalization

URDF has no capsule primitive, so the xacro source emits cylinders with the
same total end-to-end length as the intended roller capsules. After every
`urdf-to-mjcf` command, run the checked postprocessor on the generated file:

```bash
python3 scripts/finalize_mjcf.py mjcf/galbot_one_golf.xml
python3 scripts/finalize_mjcf.py mjcf/galbot_one_golf_fixed_base.xml
python3 scripts/finalize_mjcf.py mjcf/galbot_one_golf_planar_base.xml
```

The postprocessor validates the exact 40-roller geometry contract before it
writes anything. It converts generated cylinders to local-Z MJCF capsules with
a 25 mm radius and 7.5 mm cylindrical half-length, preserves an 80 mm roller
center radius, and normalizes rigid-wheel roller orientations. Mobile variants
retain their passive local-Z hinge joints. There is no sphere or mesh fallback:
an unexpected count, position, axis, or size is an error.

Before committing regenerated assets, prove the operation is idempotent:

```bash
python3 scripts/finalize_mjcf.py --check \
  mjcf/galbot_one_golf.xml \
  mjcf/galbot_one_golf_fixed_base.xml \
  mjcf/galbot_one_golf_planar_base.xml
```

These rounded collision envelopes preserve the visible omniwheel meshes while
avoiding the thin/interpenetrating detailed contacts that destabilize the
Newton/MJWarp mobile-base solve.

## Wheeled Base

This is the default model used by the package.

```bash
urdf-to-mjcf urdf/galbot_one_golf.urdf \
  --output mjcf/galbot_one_golf.xml \
  --joint-data config/mjcf/joint_data.json \
  --appendix config/mjcf/appendix.xml \
  --collision-type mesh
```

## Planar Base

Use this when the model needs virtual planar base joints:
`base_x_joint`, `base_y_joint`, and `base_yaw_joint`.

```bash
urdf-to-mjcf urdf/galbot_one_golf_fixed_base.urdf \
  --output mjcf/galbot_one_golf_planar_base.xml \
  --no-freejoint \
  --joint-data config/mjcf/joint_data.json config/mjcf/joint_data_planar_base.json \
  --appendix config/mjcf/appendix.xml \
  --collision-type mesh
```

## Fixed Base

Use this when the model should not have a floating base or virtual planar base
joints.

```bash
urdf-to-mjcf urdf/galbot_one_golf_fixed_base.urdf \
  --output mjcf/galbot_one_golf_fixed_base.xml \
  --no-freejoint \
  --joint-data config/mjcf/joint_data.json \
  --appendix config/mjcf/appendix.xml \
  --collision-type mesh
```

## Collision Only

Use this when visual meshes are unnecessary.

```bash
urdf-to-mjcf urdf/galbot_one_golf.urdf \
  --output mjcf/galbot_one_golf.xml \
  --joint-data config/mjcf/joint_data.json \
  --appendix config/mjcf/appendix.xml \
  --collision-type mesh \
  --collision-only
```
