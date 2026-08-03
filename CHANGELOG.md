# Changelog

## Unreleased

- Use `base_link` as the robot root across Xacro, URDF, MJCF, and USD assets.
- Make `base_footprint` a direct fixed child while preserving MJCF and USD
  physical placement, joint anchors, and controller targets.
