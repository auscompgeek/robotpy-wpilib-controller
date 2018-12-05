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
