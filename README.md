# arena_omni_drive_controller

A `ros2_control` plugin for generic fixed-angle N-wheel omnidirectional chassis
(omni-wheel, mecanum, kiwi-drive, etc.).

Consumes `geometry_msgs/Twist` (or `TwistStamped`) on the controller's
reference topic, projects the body twist onto each wheel's rolling direction
to produce per-wheel velocity commands, and recovers the body twist from
measured wheel speeds via a cached pseudoinverse for `nav_msgs/Odometry`.

## Plugin

```
arena_omni_drive_controller/OmniDriveController
```

inherits `controller_interface::ControllerInterface`.

## Interfaces

| | wheel joint |
|---|---|
| command | `velocity` |
| state   | `velocity` |

N of each (any positive count), in the order declared by `wheel_joint_names`.
Joint indices must align with `wheel_positions_x`, `wheel_positions_y`, and
`wheel_angles`.

## Math

For wheel `i` at body-frame position `(xᵢ, yᵢ)` with rolling-axis angle `θᵢ`
and shared radius `R`:

```
ωᵢ · R = cos θᵢ · vx + sin θᵢ · vy + (xᵢ sin θᵢ − yᵢ cos θᵢ) · ωz
```

Stacked into `M ∈ ℝ^{N×3}`: inverse kinematics is `ω = M · v`, forward
kinematics is `v_est = M⁺ · ω_measured` via cached `ColPivHouseholderQR(M)`.
`M` is static, built once in `on_configure`. Rank-deficient geometries (e.g.,
diff-drive: 2 wheels with `θ = 0`) drop the unobservable DoF in IK and odom,
and log a one-shot warn at configure time naming the lost axis.

## Parameters

See [`src/arena_omni_drive_controller_parameters.yaml`](src/arena_omni_drive_controller_parameters.yaml).
Minimum required: `wheel_joint_names`, `wheel_positions_x`, `wheel_positions_y`,
`wheel_angles`, `wheel_radius`. All four per-wheel arrays must have the same
non-zero length.

## Speed limits

Each body axis (`linear.x`, `linear.y`, `angular.z`) has an optional limiter applied to the
commanded twist before the kinematics, with the semantics of `diff_drive_controller`:

| parameter (under `<axis>.`) | default | meaning |
|---|---|---|
| `has_velocity_limits` | `false` | clamp the command to `[min_velocity, max_velocity]` |
| `has_acceleration_limits` | `false` | clamp the change per update against the previously applied command to `[min_acceleration, max_acceleration] * period` |
| `min_velocity`, `max_velocity` | `.nan` | NaN `min_velocity` mirrors `-max_velocity`, NaN `max_velocity` is unbounded |
| `min_acceleration`, `max_acceleration` | `.nan` | NaN `min_acceleration` mirrors `-max_acceleration`, NaN `max_acceleration` is unbounded |

The previous limited command resets on activation. A command older than `cmd_vel_timeout`
is zeroed without acceleration limiting.

```yaml
linear.x.has_velocity_limits: true
linear.x.max_velocity: 0.8
linear.x.has_acceleration_limits: true
linear.x.max_acceleration: 1.5
```

## Topics

| topic | direction | type | when |
|---|---|---|---|
| `~/cmd_vel`   | sub | `geometry_msgs/Twist`        | `use_stamped_vel: false` |
| `~/cmd_vel`   | sub | `geometry_msgs/TwistStamped` | `use_stamped_vel: true`  |
| `~/odometry`  | pub | `nav_msgs/Odometry`          | `publish_odom: true`     |
| `/tf`         | pub | `tf2_msgs/TFMessage`         | `publish_tf: true`       |

## Example: 3-omni kicker bot (120° spacing, rolling tangent to radial)

```yaml
/**:
  controller_manager:
    ros__parameters:
      update_rate: 50
      joint_state_broadcaster:
        type: joint_state_broadcaster/JointStateBroadcaster
      omni_velocity_controller:
        type: arena_omni_drive_controller/OmniDriveController

  omni_velocity_controller:
    ros__parameters:
      wheel_joint_names: [wheel1_speed, wheel2_speed, wheel3_speed]
      wheel_positions_x: [ 0.0236,  0.0236, -0.0472]
      wheel_positions_y: [ 0.0409, -0.0409,  0.0   ]
      # rolling-axis angles in body frame (radians), tangent to radial
      wheel_angles:      [ 2.6180,  0.5236,  4.7124]
      wheel_radius:      0.0285
      cmd_vel_timeout:   0.5
      use_stamped_vel:   true
      publish_odom:      true
      publish_tf:        false
      odom_frame_id:     odom
      base_frame_id:     base
```

## Example: 4-omni cardinal (rolling along ±X, ±Y)

```yaml
omni_velocity_controller:
  ros__parameters:
    wheel_joint_names: [front_wheel, left_wheel, rear_wheel, right_wheel]
    wheel_positions_x: [ 0.15,  0.00, -0.15,  0.00]
    wheel_positions_y: [ 0.00,  0.15,  0.00, -0.15]
    wheel_angles:      [ 1.5708, 3.1416, -1.5708, 0.0]
    wheel_radius:      0.05
```

## Example: 4-wheel mecanum (rollers at ±π/4 baked into θ)

```yaml
omni_velocity_controller:
  ros__parameters:
    wheel_joint_names: [wheel_fl, wheel_fr, wheel_rl, wheel_rr]
    wheel_positions_x: [ 0.3,  0.3, -0.3, -0.3]
    wheel_positions_y: [ 0.2, -0.2,  0.2, -0.2]
    # wheel axis along ±X (= θ_wheel 0) plus roller offset ±π/4 -> θ effective
    wheel_angles:      [ 0.7854, -0.7854, -0.7854, 0.7854]
    wheel_radius:      0.1
```
