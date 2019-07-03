import time
from unittest.mock import Mock

import pytest

from wpilib_controller import PIDController, PIDControllerRunner

input_range = 200


@pytest.fixture(scope="function")
def pid_controller():
    controller = PIDController(0.05, 0.0, 0.0)
    controller.setInputRange(-input_range / 2, input_range / 2)
    yield controller
    controller.close()


@pytest.fixture(scope="function")
def pid_runner(pid_controller: PIDController):
    inp = Mock(return_value=0)
    out = Mock()
    runner = PIDControllerRunner(pid_controller, inp, out)
    yield runner
    runner.disable()
    runner.notifier.close()


def test_absolute_tolerance(
    pid_controller: PIDController, pid_runner: PIDControllerRunner
):
    setpoint = 50
    tolerance = 10
    inp = pid_runner.measurement_source

    pid_controller.setAbsoluteTolerance(tolerance)
    pid_controller.setSetpoint(setpoint)
    pid_runner.enable()
    time.sleep(1)
    assert not pid_controller.atSetpoint(), (
        "Error was in tolerance when it should not have been. Error was %f"
        % pid_controller.getError()
    )

    inp.return_value = setpoint + tolerance / 2
    time.sleep(1)
    assert pid_controller.atSetpoint(), (
        "Error was not in tolerance when it should have been. Error was %f"
        % pid_controller.getError()
    )

    inp.return_value = setpoint + 10 * tolerance
    time.sleep(1)
    assert not pid_controller.atSetpoint(), (
        "Error was in tolerance when it should not have been. Error was %f"
        % pid_controller.getError()
    )


def test_percent_tolerance(pid_controller: PIDController, pid_runner: PIDControllerRunner):
    setpoint = 50
    tolerance = 10
    inp = pid_runner.measurement_source

    pid_controller.setPercentTolerance(tolerance)
    pid_controller.setSetpoint(setpoint)
    pid_runner.enable()
    assert not pid_controller.atSetpoint(), (
        "Error was in tolerance when it should not have been. Error was %f"
        % pid_controller.getError()
    )

    # half of percent tolerance away from setpoint
    inp.return_value = setpoint + tolerance / 200 * input_range
    time.sleep(1)
    assert pid_controller.atSetpoint(), (
        "Error was not in tolerance when it should have been. Error was %f"
        % pid_controller.getError()
    )

    # double percent tolerance away from setpoint
    inp.return_value = setpoint + tolerance / 50 * input_range
    time.sleep(1)
    assert not pid_controller.atSetpoint(), (
        "Error was in tolerance when it should not have been. Error was %f"
        % pid_controller.getError()
    )
