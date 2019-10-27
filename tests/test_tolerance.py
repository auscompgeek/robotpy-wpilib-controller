import pytest

from wpilib_controller import PIDController

input_range = 200


@pytest.fixture(scope="function")
def pid_controller():
    controller = PIDController(0.05, 0.0, 0.0)
    controller.enableContinuousInput(-input_range / 2, input_range / 2)
    yield controller
    controller.close()


def test_absolute_tolerance(pid_controller: PIDController):
    setpoint = 50
    tolerance = 10

    pid_controller.setTolerance(tolerance)
    pid_controller.setSetpoint(setpoint)

    pid_controller.calculate(0)

    assert not pid_controller.atSetpoint(), (
        "Error was in tolerance when it should not have been. Error was %f"
        % pid_controller.getPositionError()
    )

    measurement = setpoint + tolerance / 2
    pid_controller.calculate(measurement)

    assert pid_controller.atSetpoint(), (
        "Error was not in tolerance when it should have been. Error was %f"
        % pid_controller.getPositionError()
    )

    measurement = setpoint + 10 * tolerance
    pid_controller.calculate(measurement)

    assert not pid_controller.atSetpoint(), (
        "Error was in tolerance when it should not have been. Error was %f"
        % pid_controller.getPositionError()
    )
