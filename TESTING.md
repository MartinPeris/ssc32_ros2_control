# Test strategy

The automated tests deliberately cross the package boundaries that are most
likely to hide integration defects.

## Layers

1. Calibration tests prove SI-position/pulse conversion and both position and
   pulse envelopes.
2. Driver tests use an in-memory transport to verify exact protocol bytes,
   batching, and duplicate suppression.
3. Serial tests use a POSIX pseudoterminal and the real `ssc32::SerialTransport`.
4. Hardware-system tests exercise lifecycle, exported interfaces, reads, writes,
   timeouts, truncated replies, disconnection, and invalid reported pulses.
5. The controller-manager test loads the plugin and the standard
   `joint_trajectory_controller`, sends a `FollowJointTrajectory` action, runs
   the control loop, and verifies the resulting multi-channel SSC-32 traffic.
6. Configuration tests prove that the plugin and inspection tool share one
   parser, including multi-board selection and JSON/text rendering.
7. Diagnostic tests exercise lifecycle levels, stale detection, counters,
   errors, and actual publication on `/diagnostics`.

The pseudoterminal emulator is a stateful model of the protocol boundary, not a
servo-physics model. It updates generated pulse outputs when it accepts a move
and can deterministically delay, drop, truncate, or disconnect the next reply.
It cannot validate linkage geometry, loading, electrical behavior, servo
tracking, backlash, or collision response.

## Serial timing envelope

SSC-32 serial uses ten wire bits per byte with 8N1 framing. A representative
six-channel move such as six `#n P1500 S100 ` segments plus carriage return is
85 bytes. Its transmit-only lower bound is approximately:

- 7.4 ms at 115200 baud (about 135 writes/s before USB and board latency).
- 88.5 ms at 9600 baud (about 11 writes/s before latency).

A six-channel pulse query is 19 request bytes plus 6 response bytes, giving a
wire-only round trip of roughly 2.2 ms at 115200 baud or 26 ms at 9600 baud.
Because every hardware read performs that query, a 50 Hz hardware rate is a
reasonable starting point at 115200 baud but cannot be sustained at 9600 baud.
These are framing bounds, not measured end-to-end guarantees; USB scheduling,
controller processing, host load, and timeout policy add latency.

## Local Lyrical matrix

Install the repository's pre-commit hook and run the complete gate manually:

```sh
./scripts/install_git_hooks.sh
./scripts/quality_gate.sh
```

The versioned hook validates the exact staged Git tree in an isolated ROS 2
Lyrical container. The gate checks clang-format and shell scripts; runs
clang-tidy; builds with GCC and Clang warnings-as-errors; executes all unit,
serial, lifecycle, diagnostics, and controller-manager tests; runs ASan/UBSan;
and enforces production coverage floors of 95% lines and 75% branches. The
Docker image and compiler cache are reused locally, while all source/build
workspaces are disposable.

CI independently repeats the GCC, Clang, AddressSanitizer, and
UndefinedBehaviorSanitizer matrix as a final repository-side safeguard.
