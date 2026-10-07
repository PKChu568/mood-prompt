# Kinematics data attribution

`kinematics_data.json` is vendored from Pollen Robotics' Reachy Mini SDK:

- Source: https://github.com/pollen-robotics/reachy_mini
  (`src/reachy_mini/assets/kinematics_data.json`)
- License: Apache License 2.0
- Copyright: Pollen Robotics

It defines the Stewart-platform geometry (motor arm length, rod length,
per-motor world transforms and branch attachment points, the head Z
offset, and the assembly-mode "solution" sign per leg) that the mood
trajectories' head poses are authored against. We use it as the
authoritative geometry so our inverse kinematics applies head poses in
the same frame convention as the dataset.
