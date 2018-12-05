"""A backport of the upcoming (in 2020) WPILib PIDController."""

__version__ = "0.2"

import enum
import math
import threading

from typing import Any, Callable, ClassVar
from typing_extensions import Protocol

import wpilib

__any__ = ("ControllerRunner", "PIDController", "Controller", "MeasurementSource")


class Controller(Protocol):
    """
    Interface for Controllers.

    Controllers run control loops, the most common are PID controllers
    and their variants, but this includes anything that is controlling
    an actuator in a separate thread.
    """

    period: float

    def update(self) -> float:
        """Read the input, calculate the output accordingly, and return the output."""
        ...


class ControllerRunner:
    notifier: wpilib.Notifier
    _enabled: bool = False

    def __init__(
        self, controller: Controller, controller_output: Callable[[float], Any]
    ) -> None:
        self.controller_update = controller.update
        self.controller_output = controller_output

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
            mutex_owned = self._this_mutex.acquire()
            try:
                if self._enabled:
                    # Don't block other ControllerRunner operations on output
                    self._this_mutex.release()
                    mutex_owned = False

                    self.controller_output(self.controller_update())
            finally:
                if mutex_owned:
                    self._this_mutex.release()


MeasurementSource = Callable[[], float]


class PIDController(wpilib.SendableBase):
    """Class implements a PID Control Loop.

    This feedback controller runs in discrete time, so time deltas are not used
    in the integral and derivative calculations.  Therefore, the sample rate
    affects the controller's behavior for a given set of PID constants.
    """

    instances: ClassVar[int] = 0

    period: float

    #: Factor for "proportional" control
    Kp: float
    #: Factor for "integral" control
    Ki: float
    #: Factor for "derivative" control
    Kd: float

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

    #: The prior error (used to compute velocity)
    prev_error: float = 0
    #: The sum of the errors for use in the integral calc
    total_error: float = 0

    class Tolerance(enum.Enum):
        Absolute = enum.auto()
        Percent = enum.auto()

    _tolerance_type: Tolerance = Tolerance.Absolute

    #: The percentage or absolute error that is considered at reference.
    _tolerance: float = 0.05
    _delta_tolerance: float = math.inf

    reference: float = 0
    output: float = 0

    _this_mutex: threading.RLock

    def __init__(
        self,
        Kp: float,
        Ki: float,
        Kd: float,
        *,
        feedforward: Callable[[], float] = lambda: 0,
        measurement_source: MeasurementSource,
        period: float = 0.05,
    ) -> None:
        """Allocate a PID object with the given constants for Kp, Ki, and Kd.

        :param Kp: The proportional coefficient.
        :param Ki: The integral coefficient.
        :param Kd: The derivative coefficient.
        :param feedforward: The arbitrary feedforward function.
        :param measurement_source: The function used to take measurements.
        :param period: The period between controller updates in seconds.
                       The default is 50ms.
        """
        self._this_mutex = threading.RLock()

        self.period = period
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.feedforward = feedforward
        self.measurement_source = measurement_source

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

    def setReference(self, reference: float) -> None:
        """Set the reference for the PIDController."""
        with self._this_mutex:
            if self.maximum_input > self.minimum_input:
                self.reference = self._clamp(
                    reference, self.minimum_input, self.maximum_input
                )
            else:
                self.reference = reference

    def atReference(self) -> bool:
        """
        Return true if the error is within the percentage of the total input range, determined by setTolerance.

        This asssumes that the maximum and minimum input were set using setInput.

        Currently this just reports on target as the actual value passes through the setpoint.
        Ideally it should be based on being within the tolerance for some period of time.

        This will return false until at least one input value has been computed.
        """
        error = self.getError()

        with self._this_mutex:
            delta_error = (error - self.prev_error) / self.period
            if self._tolerance_type is self.Tolerance.Percent:
                input_range = self.input_range
                return (
                    abs(error) < self._tolerance / 100 * input_range
                    and abs(delta_error) < self._delta_tolerance / 100 * input_range
                )
            else:
                return (
                    abs(error) < self._tolerance
                    and abs(delta_error) < self._delta_tolerance
                )

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

        self.setReference(self.reference)

    def setAbsoluteTolerance(
        self, tolerance: float, delta_tolerance: float = math.inf
    ) -> None:
        """
        Set the absolute error which is considered tolerable for use with atReference().

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
        Set the percent error which is considered tolerable for use with atReference().

        :param tolerance: Percent error which is tolerable.
        :param delta_tolerance: Change in percent error per second which is tolerable.
        """
        with self._this_mutex:
            self._tolerance_type = self.Tolerance.Percent
            self._tolerance = tolerance
            self._delta_tolerance = delta_tolerance

    def getError(self) -> float:
        """Returns the difference between the reference and the measurement."""
        with self._this_mutex:
            return self.getContinuousError(self.reference - self.measurement_source())

    def getDeltaError(self) -> float:
        """Returns the change in error per second."""
        error = self.getError()
        with self._this_mutex:
            return (error - self.prev_error) / self.period

    def update(self) -> float:
        feedforward = self.feedforward()
        measurement = self.measurement_source()

        with self._this_mutex:
            Kp = self.Kp
            Ki = self.Ki
            Kd = self.Kd
            minimum_output = self.minimum_output
            maximum_output = self.maximum_output

            prev_error = self.prev_error
            error = self.getContinuousError(self.reference - measurement)
            total_error = self.total_error

            period = self.period

        if Ki:
            total_error = self._clamp(
                total_error + error * period, minimum_output / Ki, maximum_output / Ki
            )

        output = self._clamp(
            Kp * error
            + Ki * total_error
            + Kd * (error - prev_error) / period
            + feedforward,
            minimum_output,
            maximum_output,
        )

        with self._this_mutex:
            self.prev_error = error
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
        builder.addDoubleProperty("Kp", lambda: self.Kp, self.setP)
        builder.addDoubleProperty("Ki", lambda: self.Ki, self.setI)
        builder.addDoubleProperty("Kd", lambda: self.Kd, self.setD)
        builder.addDoubleProperty(
            "feedforward",
            self.feedforward,
            lambda x: setattr(self, "feedforward", lambda: x),
        )
        builder.addDoubleProperty(
            "reference", lambda: self.reference, self.setReference
        )

    def getContinuousError(self, error: float) -> float:
        """Wraps error around for continuous inputs.

        The original error is returned if continuous mode is disabled.
        This is an unsynchronized function.

        :param error: The current error of the PID controller.
        :return: Error for continuous inputs.
        """
        input_range = self._input_range
        if self.continuous and input_range > 0:
            error = math.fmod(error, input_range)
            if abs(error) > input_range / 2:
                if error > 0:
                    return error - input_range
                else:
                    return error + input_range

        return error

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(value, high))
