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
    #: input range - difference between maximum and minimum
    _input_range: float = 0
    #: Do the endpoints wrap around? eg. Absolute encoder
    continuous: bool = False

    #: The error at the time of the most recent call to calculate()
    curr_error: float = 0
    #: The error at the time of the second-most-recent call to calculate() (used to compute velocity)
    prev_error: float = math.inf
    #: The sum of the errors for use in the integral calc
    total_error: float = 0

    class Tolerance(enum.Enum):
        Absolute = enum.auto()
        Percent = enum.auto()

    _tolerance_type: Tolerance = Tolerance.Absolute

    #: The percentage or absolute error that is considered at setpoint.
    _tolerance: float = 0.05
    _delta_tolerance: float = math.inf

    setpoint: float = 0
    output: float = 0

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
        tolerance: Optional[float] = None,
        delta_tolerance: float = math.inf,
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
        if tolerance is None:
            tolerance = self._tolerance
            delta_tolerance = self._delta_tolerance
            tolerance_type = self._tolerance_type

        error = self.getError()

        delta_error = (error - self.prev_error) / self.period
        if tolerance_type is self.Tolerance.Percent:
            input_range = self._input_range
            return (
                abs(error) < tolerance / 100 * input_range
                and abs(delta_error) < delta_tolerance / 100 * input_range
            )
        else:
            return abs(error) < tolerance and abs(delta_error) < delta_tolerance

    def setContinuous(self, continuous: bool = True) -> None:
        """Set the PID controller to consider the input to be continuous.

        Rather than using the max and min input range as constraints, it
        considers them to be the same point and automatically calculates
        the shortest route to the setpoint.

        :param continuous: True turns on continuous, False turns off continuous
        """
        self.continuous = continuous

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

    def setOutputRange(self, minimum_output: float, maximum_output: float) -> None:
        """Sets the minimum and maximum values to write.

        :param minimum_output: the minimum value to write to the output
        :param maximum_output: the maximum value to write to the output
        """
        self.minimum_output = minimum_output
        self.maximum_output = maximum_output

    def setAbsoluteTolerance(
        self, tolerance: float, delta_tolerance: float = math.inf
    ) -> None:
        """
        Set the absolute error which is considered tolerable for use with atSetpoint().

        :param tolerance: Absolute error which is tolerable.
        :param delta_tolerance: Change in absolute error per second which is tolerable.
        """
        self._tolerance_type = self.Tolerance.Absolute
        self._tolerance = tolerance
        self._delta_tolerance = delta_tolerance

    def setPercentTolerance(
        self, tolerance: float, delta_tolerance: float = math.inf
    ) -> None:
        """
        Set the percent error which is considered tolerable for use with atSetpoint().

        :param tolerance: Percent error which is tolerable.
        :param delta_tolerance: Change in percent error per second which is tolerable.
        """
        self._tolerance_type = self.Tolerance.Percent
        self._tolerance = tolerance
        self._delta_tolerance = delta_tolerance

    def getError(self) -> float:
        """Returns the difference between the setpoint and the measurement."""
        return self.getContinuousError(self.curr_error)

    def getDeltaError(self) -> float:
        """Returns the change in error per second."""
        return (self.getError() - self.prev_error) / self.period

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

        prev_error = self.prev_error = self.curr_error
        error = self.curr_error = self.getContinuousError(self.setpoint - measurement)
        total_error = self.total_error

        period = self.period

        if Ki:
            total_error = self.total_error = self._clamp(
                total_error + error * period, minimum_output / Ki, maximum_output / Ki
            )

        output = self.output = self._clamp(
            self.Kp * error
            + Ki * total_error
            + self.Kd * (error - prev_error) / period,
            minimum_output,
            maximum_output,
        )

        return output

    def reset(self) -> None:
        """Reset the previous error, the integral term, and disable the controller."""
        self.prev_error = 0
        self.total_error = 0
        self.output = 0

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
