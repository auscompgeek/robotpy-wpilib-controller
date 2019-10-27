"""A backport of the upcoming (in 2020) WPILib PIDController."""

__version__ = "0.6"

import enum
import math

from typing import ClassVar, Optional

import wpilib

__any__ = ("PIDController",)


class PIDController(wpilib.SendableBase):
    """Class implements a PID Control Loop."""

    instances: ClassVar[int] = 0

    #: Factor for "proportional" control
    Kp: float
    #: Factor for "integral" control
    Ki: float
    #: Factor for "derivative" control
    Kd: float

    #: The period (in seconds) of the loop that calls the controller
    period: float

    maximum_output: float = 1
    minimum_output: float = -1
    #: Maximum input - limit setpoint to this
    _maximum_input: float = 0
    #: Minimum input - limit setpoint to this
    _minimum_input: float = 0
    #: Input range - difference between maximum and minimum
    _input_range: float = 0
    #: Do the endpoints wrap around? eg. Absolute encoder
    continuous: bool = False

    #: The error at the time of the most recent call to calculate()
    _position_error: float = 0
    _velocity_error: float = 0

    #: The error at the time of the second-most-recent call to calculate() (used to compute velocity)
    prev_error: float = math.inf

    #: The sum of the errors for use in the integral calc
    total_error: float = 0

    class Tolerance(enum.Enum):
        Absolute = enum.auto()
        Percent = enum.auto()

    _tolerance_type: Tolerance = Tolerance.Absolute

    #: The percentage or absolute error that is considered at setpoint.
    _position_tolerance: float = 0.05
    _velocity_tolerance: float = math.inf

    setpoint: float = 0

    def __init__(
        self, Kp: float, Ki: float, Kd: float, *, period: float = 0.02
    ) -> None:
        """Allocate a PID object with the given constants for Kp, Ki, and Kd.

        :param Kp: The proportional coefficient.
        :param Ki: The integral coefficient.
        :param Kd: The derivative coefficient.
        :param period: The period between controller updates in seconds.
                       The default is 20ms.
        """
        super().__init__(addLiveWindow=False)

        self.period = period
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        PIDController.instances += 1
        self.setName("PIDController", PIDController.instances)

    def setPID(self, Kp: float, Ki: float, Kd: float) -> None:
        """Set the PID Controller gain parameters."""
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def setP(self, Kp: float) -> None:
        """Set the Proportional coefficient of the PID controller gain."""
        self.Kp = Kp

    def setI(self, Ki: float) -> None:
        """Set the Integral coefficient of the PID controller gain."""
        self.Ki = Ki

    def setD(self, Kd: float) -> None:
        """Set the Differential coefficient of the PID controller gain."""
        self.Kd = Kd

    def setSetpoint(self, setpoint: float) -> None:
        """Set the setpoint for the PIDController."""
        if self._maximum_input > self._minimum_input:
            self.setpoint = self._clamp(
                setpoint, self._minimum_input, self._maximum_input
            )
        else:
            self.setpoint = setpoint

    def atSetpoint(
        self,
        position_tolerance: Optional[float] = None,
        velocity_tolerance: float = math.inf,
        tolerance_type: Tolerance = Tolerance.Absolute,
    ) -> bool:
        """
        Return true if the error is within the percentage of the specified tolerances.

        This will return false until at least one input value has been computed.

        If no arguments are given, defaults to the tolerances set by
        :meth:`setAbsoluteTolerance` or :meth:`setPercentTolerance`.

        :param tolerance: The maximum allowable error.
        :param delta_tolerance: The maximum allowable change in error, if tolerance is specified.
        :param tolerance_type: The type of tolerances specified.
        """
        if position_tolerance is None:
            position_tolerance = self._position_tolerance
            velocity_tolerance = self._velocity_tolerance
            tolerance_type = self._tolerance_type

        if tolerance_type is self.Tolerance.Percent:
            input_range = self._input_range
            return (
                abs(self._position_error) < position_tolerance / 100 * input_range
                and abs(self._velocity_error) < velocity_tolerance / 100 * input_range
            )
        else:
            return (
                abs(self._position_error) < position_tolerance
                and abs(self._velocity_error) < velocity_tolerance
            )

    def setInputRange(self, minimum_input: float, maximum_input: float) -> None:
        """Sets the maximum and minimum values expected from the input.

        :param minimum_input: The minimum value expected from the input.
        :param maximum_input: The maximum value expected from the input.
        """
        self._minimum_input = minimum_input
        self._maximum_input = maximum_input
        self._input_range = maximum_input - minimum_input

        # Clamp setpoint to new input
        if maximum_input > minimum_input:
            self.setpoint = self._clamp(self.setpoint, minimum_input, maximum_input)

    def enableContinuousInput(self, minimum_input: float, maximum_input: float) -> None:
        """Enable continuous input.

        Rather than using the max and min input range as constraints, it
        considers them to be the same point and automatically calculates
        the shortest route to the setpoint.

        :param minimum_input: The minimum value expected from the input.
        :param maximum_input: The maximum value expected from the input.
        """
        self.continuous = True
        self.setInputRange(minimum_input, maximum_input)

    def disableContinuousInput(self) -> None:
        """Disables continuous input."""
        self.continuous = False

    def setOutputRange(self, minimum_output: float, maximum_output: float) -> None:
        """Sets the minimum and maximum values to write.

        :param minimum_output: the minimum value to write to the output
        :param maximum_output: the maximum value to write to the output
        """
        self.minimum_output = minimum_output
        self.maximum_output = maximum_output

    def setAbsoluteTolerance(
        self, position_tolerance: float, velocity_tolerance: float = math.inf
    ) -> None:
        """
        Set the absolute error which is considered tolerable for use with atSetpoint().

        :param position_tolerance: Position error which is tolerable.
        :param velocity_tolerance: Velocity error which is tolerable.
        """
        self._tolerance_type = self.Tolerance.Absolute
        self._position_tolerance = position_tolerance
        self._velocity_tolerance = velocity_tolerance

    def setPercentTolerance(
        self, position_tolerance: float, velocity_tolerance: float = math.inf
    ) -> None:
        """
        Set the percent error which is considered tolerable for use with atSetpoint().

        :param position_tolerance: Position error which is tolerable.
        :param velocity_tolerance: Velocity error which is tolerable.
        """
        self._tolerance_type = self.Tolerance.Percent
        self._position_tolerance = position_tolerance
        self._velocity_tolerance = velocity_tolerance

    def getPositionError(self) -> float:
        """Returns the difference between the setpoint and the measurement."""
        return self.getContinuousError(self._position_error)

    def getVelocityError(self) -> float:
        """Returns the velocity error."""
        return self._velocity_error

    def calculate(self, measurement: float, setpoint: Optional[float] = None) -> float:
        """
        Returns the next output of the PID controller.

        :param measurement: The current measurement of the process variable.
        :param setpoint: The new setpoint of the controller if specified.
        """
        if setpoint is not None:
            self.setSetpoint(setpoint)

        Ki = self.Ki
        minimum_output = self.minimum_output
        maximum_output = self.maximum_output

        prev_error = self.prev_error = self._position_error
        error = self._position_error = self.getContinuousError(
            self.setpoint - measurement
        )
        vel_error = self._velocity_error = (error - prev_error) / self.period
        total_error = self.total_error

        period = self.period

        if Ki:
            total_error = self.total_error = self._clamp(
                total_error + error * period, minimum_output / Ki, maximum_output / Ki
            )

        return self._clamp(
            self.Kp * error + Ki * total_error + self.Kd * vel_error,
            minimum_output,
            maximum_output,
        )

    def reset(self) -> None:
        """Reset the previous error, the integral term, and disable the controller."""
        self.prev_error = 0
        self.total_error = 0

    def initSendable(self, builder) -> None:
        builder.setSmartDashboardType("PIDController")
        builder.setSafeState(self.reset)
        builder.addDoubleProperty("p", lambda: self.Kp, self.setP)
        builder.addDoubleProperty("i", lambda: self.Ki, self.setI)
        builder.addDoubleProperty("d", lambda: self.Kd, self.setD)
        builder.addDoubleProperty("setpoint", lambda: self.setpoint, self.setSetpoint)

    def getContinuousError(self, error: float) -> float:
        """Wraps error around for continuous inputs.

        The original error is returned if continuous mode is disabled.

        :param error: The current error of the PID controller.
        :return: Error for continuous inputs.
        """
        input_range = self._input_range
        if self.continuous and input_range > 0:
            error %= input_range
            if error > input_range / 2:
                return error - input_range

        return error

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(value, high))
