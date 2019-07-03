"""A backport of the upcoming (in 2020) WPILib PIDController."""

__version__ = "0.5.1"

import enum
import math
import threading

from typing import Any, Callable, ClassVar, Optional

import wpilib

__any__ = ("PIDController", "PIDControllerRunner", "MeasurementSource")


class PIDControllerRunner(wpilib.SendableBase):
    def __init__(
        self,
        controller: "PIDController",
        measurement_source: Callable[[], float],
        controller_output: Callable[[float], Any],
    ) -> None:
        """
        Allocates a PIDControllerRunner.

        :param measurement_source: The function that supplies the current process variable measurement.
        :param controller_output: The function which updates the plant using the controller output
                                  passed as the argument.
        """
        super().__init__()
        self._enabled = False

        self.controller = controller
        self.controller_output = controller_output
        self.measurement_source = measurement_source

        self._this_mutex = threading.RLock()

        # Ensures when disable() is called, self.controller_output() won't
        # run if Controller.update() is already running at that time.
        self._output_mutex = threading.RLock()

        self.notifier = wpilib.Notifier(self._run)
        self.notifier.startPeriodic(controller.period)

    def enable(self):
        """Begin running the controller."""
        with self._this_mutex:
            self._enabled = True

    def disable(self):
        """Stop running the controller.

        This sets the output to zero before stopping.
        """
        # Ensures self._enabled modification and self.controller_output()
        # call occur atomically
        with self._output_mutex:
            with self._this_mutex:
                self._enabled = False
            self.controller_output(0)

    def isEnabled(self):
        """Returns whether controller is running."""
        with self._this_mutex:
            return self._enabled

    def _run(self):
        # Ensures self._enabled check and self.controller_output() call occur atomically
        with self._output_mutex:
            with self._this_mutex:
                enabled = self._enabled
            if enabled:
                self.controller_output(
                    self.controller.calculate(self.measurement_source())
                )

    def initSendable(self, builder) -> None:
        self.controller.initSendable(builder)
        builder.setSafeState(self.disable)
        builder.addBooleanProperty(
            "enabled",
            self.isEnabled,
            lambda enabled: self.enable() if enabled else self.disable(),
        )


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

    _this_mutex: threading.RLock

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
        self._this_mutex = threading.RLock()

        self.period = period
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        PIDController.instances += 1
        self.setName("PIDController", PIDController.instances)

    def setPID(self, Kp: float, Ki: float, Kd: float) -> None:
        """Set the PID Controller gain parameters."""
        with self._this_mutex:
            self.Kp = Kp
            self.Ki = Ki
            self.Kd = Kd

    def setP(self, Kp: float) -> None:
        """Set the Proportional coefficient of the PID controller gain."""
        with self._this_mutex:
            self.Kp = Kp

    def setI(self, Ki: float) -> None:
        """Set the Integral coefficient of the PID controller gain."""
        with self._this_mutex:
            self.Ki = Ki

    def setD(self, Kd: float) -> None:
        """Set the Differential coefficient of the PID controller gain."""
        with self._this_mutex:
            self.Kd = Kd

    def setSetpoint(self, setpoint: float) -> None:
        """Set the setpoint for the PIDController."""
        with self._this_mutex:
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

        This asssumes that the maximum and minimum input were set using setInput.

        This will return false until at least one input value has been computed.

        If no arguments are given, defaults to the tolerances set by setTolerance.

        :param tolerance: The maximum allowable error.
        :param delta_tolerance: The maximum allowable change in error, if tolerance is specified.
        :param tolerance_type: The type of tolerances specified.
        """
        if tolerance is None:
            tolerance = self._tolerance
            delta_tolerance = self._delta_tolerance
            tolerance_type = self._tolerance_type

        error = self.getError()

        with self._this_mutex:
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
        with self._this_mutex:
            self.continuous = continuous

    def setInputRange(self, minimum_input: float, maximum_input: float) -> None:
        """Sets the maximum and minimum values expected from the input.

        :param minimumInput: the minimum value expected from the input
        :param maximumInput: the maximum value expected from the output
        """
        with self._this_mutex:
            self._minimum_input = minimum_input
            self._maximum_input = maximum_input
            self._input_range = maximum_input - minimum_input

        self.setSetpoint(self.setpoint)

    def setOutputRange(self, minimum_output: float, maximum_output: float) -> None:
        """Sets the minimum and maximum values to write."""
        with self._this_mutex:
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
        with self._this_mutex:
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
        with self._this_mutex:
            self._tolerance_type = self.Tolerance.Percent
            self._tolerance = tolerance
            self._delta_tolerance = delta_tolerance

    def getError(self) -> float:
        """Returns the difference between the setpoint and the measurement."""
        with self._this_mutex:
            return self.getContinuousError(self.curr_error)

    def getDeltaError(self) -> float:
        """Returns the change in error per second."""
        error = self.getError()
        with self._this_mutex:
            return (error - self.prev_error) / self.period

    def calculate(self, measurement: float, setpoint: Optional[float] = None) -> float:
        """
        Calculates the output of the PID controller.

        :param measurement: The current measurement of the process variable.
        :param setpoint: The setpoint of the controller if specified.
        :returns: The controller output.
        """
        if setpoint is not None:
            self.setSetpoint(setpoint)

        with self._this_mutex:
            Kp = self.Kp
            Ki = self.Ki
            Kd = self.Kd
            minimum_output = self.minimum_output
            maximum_output = self.maximum_output

            prev_error = self.prev_error = self.curr_error
            error = self.curr_error = self.getContinuousError(self.setpoint - measurement)
            total_error = self.total_error

            period = self.period

        if Ki:
            total_error = self._clamp(
                total_error + error * period, minimum_output / Ki, maximum_output / Ki
            )

        output = self._clamp(
            Kp * error + Ki * total_error + Kd * (error - prev_error) / period,
            minimum_output,
            maximum_output,
        )

        with self._this_mutex:
            self.total_error = total_error
            self.output = output

        return output

    def reset(self) -> None:
        """Reset the previous error, the integral term, and disable the controller."""
        with self._this_mutex:
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
        This is an unsynchronized function.

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
