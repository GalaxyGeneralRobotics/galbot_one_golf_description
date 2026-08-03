# Changelog

## Unreleased

- Use `base_link` as the robot root across Xacro, URDF, MJCF, and USD assets.
- Make `base_footprint` a direct fixed child while preserving MJCF and USD
  physical placement, joint anchors, and controller targets.
- Add an optional USD `ROS=ros2` variant for namespaced base commands,
  whole-body and gripper joint commands, joint-state feedback, and simulation
  clock publishing. The default `ROS=none` variant has no ROS dependency.
