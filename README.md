# wpilib-controller

A backport of the upcoming (in 2020) WPILib PIDController.

This is a backport of [wpilibsuite/allwpilib#1300][], which is planned to be
merged for the 2020 season, for [RobotPy][].

@calcmogul has some [in-progress docs][] on this.

Note that if you are moving from the old WPILib PIDController, your PID
constants will need to change, as it did not consider the discretization period:

- divide your Ki gain by 0.05, and
- multiply your Kd gain by 0.05,
- where 0.05 is the original default period (use the period you used otherwise).

[wpilibsuite/allwpilib#1300]: https://github.com/wpilibsuite/allwpilib/pull/1300
[RobotPy]: https://robotpy.github.io
[in-progress docs]: https://docs.google.com/document/d/1M6sCsqxvQtP2qSIYkvfMb-k3sdPfEg9gsKKneY6i-os/view

## Installation

For a computer with an internet connection:

```
pip install wpilib-controller
```

To install onto an offline roboRIO:

<!--
NOTE: robotpy-installer does not download wheels:
https://github.com/robotpy/robotpy-installer/issues/24
-->
```bash
pip download wpilib-controller -d pip_cache  # with an internet connection
robotpy-installer install-pip wpilib-controller  # connected to your robot
```

## Example

```python
from wpilib_controller import PIDController

# assume gyro is a gyro object created prior
controller = PIDController(1, 0, 0, measurement_source=gyro.getAngle)
controller.setInputRange(0, 360)
controller.setContinuous()

# setSetpoint is now called setReference
controller.setReference(180)

# elsewhere...

output = controller.update()
# do something with the output, for example:
motor.set(output)
```

## Major differences

- The PID gains are no longer time dependent.
- This does not distinguish between displacement and rate “PID source types”. Instead this PIDController takes a function as its measurement source.
- The feedforward is calculated by a function passed into the PIDController.
- This provides a simple way of running a PID controller synchronously in your robot code (as opposed to having it run in a thread). Simply call the `update()` method which will give you the controller output.
