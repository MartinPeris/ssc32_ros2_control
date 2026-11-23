# Contributing

Keep protocol behavior in the `ssc32` library and ROS lifecycle behavior here.
Changes should include deterministic tests that do not require physical
hardware. Hardware reports should state controller model and firmware, USB
serial adapter, baud rate, ROS distribution, and the smallest safe reproduction.

Install the versioned Git hook once per clone:

```sh
./scripts/install_git_hooks.sh
```

Every commit then runs the complete Dockerized quality gate against the exact
staged snapshot. Run the same gate explicitly against current working-tree
content with `./scripts/quality_gate.sh`.
