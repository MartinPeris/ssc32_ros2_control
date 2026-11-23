# Architecture

## Boundaries

- `ssc32`: protocol encoding, bounded serial transport, and controller queries.
- `Driver`: calibrated SI-position/pulse conversion and atomic command batching.
- `Ssc32System`: ros2_control lifecycle and position command/state interfaces.
- Standard ROS controllers: trajectory interpolation, action handling, and joint
  state publication.

No SSC-32 command encoder is duplicated in this package.

## Lifecycle

- `on_init` validates the complete board and joint configuration without opening
  a device.
- `on_configure` opens serial, reads firmware, queries all pulse outputs, and
  initializes state and command interfaces.
- `on_activate` queries again and re-seeds commands from current output, avoiding
  a startup jump.
- `read` performs one batched pulse query and converts outputs to SI positions.
- `write` validates all commands and sends one multi-channel move only when a
  pulse value changed.
- `on_deactivate` sends cancel but keeps the configured connection available.
- cleanup, shutdown, and error paths attempt cancel and release the connection.

## Scheduling

USB serial is not deterministic. Configure this component with Lyrical's
`is_async="true"` and an explicit `rw_rate`, normally 50 Hz, so serial latency
does not stall the controller manager's primary loop.

## Failure behavior

Configuration failure opens no serial device. A read or write exception returns
`hardware_interface::return_type::ERROR`, allowing controller-manager lifecycle
and fallback policy to react. The driver never commands a return-to-home motion
after an unknown failure.

The SSC-32 continues generating its last pulse after host failure. A physical
power cutoff remains the only dependable stop for electrical or mechanical
faults.

## Verification boundary

The test emulator models SSC-32 protocol state and deterministic transport
faults. The highest-level test loads the production hardware plugin into a real
controller manager, activates the standard trajectory controller, submits a
`FollowJointTrajectory` action, and observes the serial wire commands. This
proves the software integration without claiming servo-physics fidelity.
