from unittest.mock import MagicMock

import pytest

import wpilib
from wpilib_controller import PIDController


@pytest.fixture(scope="function")
def networktables():
    """Networktables instance"""
    import networktables

    networktables.NetworkTables.startTestMode()
    yield networktables
    networktables.NetworkTables.shutdown()


@pytest.fixture(scope="function")
def sendablebuilder(networktables):
    builder = wpilib.SendableBuilder()
    table = networktables.NetworkTables.getTable("component")
    builder.setTable(table)
    return builder


@pytest.fixture(scope="function")
def pid():
    return _get_pid()


def _get_pid():
    _pid = PIDController(Kp=1.0, Ki=0.25, Kd=0.75)
    return _pid


def test_pidcontroller_init_args1():
    pid = PIDController(1.0, 2.0, 3.0, period=5.0)

    assert pid.Kp == pytest.approx(1.0, 0.01)
    assert pid.Ki == pytest.approx(2.0, 0.01)
    assert pid.Kd == pytest.approx(3.0, 0.01)
    assert pid.period == pytest.approx(5.0, 0.01)


def test_pidcontroller_init_args0():
    pid = PIDController(period=5.0, Ki=2.0, Kp=1.0, Kd=3.0)

    assert pid.Kp == pytest.approx(1.0, 0.01)
    assert pid.Ki == pytest.approx(2.0, 0.01)
    assert pid.Kd == pytest.approx(3.0, 0.01)
    assert pid.period == pytest.approx(5.0, 0.01)


def test_pidcontroller_init_args5():
    with pytest.raises(TypeError) as exinfo:
        PIDController(Ki=2.0, Kd=3.0)

    assert (
        exinfo.value.args[0]
        == "__init__() missing 1 required positional argument: 'Kp'"
    )


def test_pidcontroller_init_args6():
    with pytest.raises(TypeError) as exinfo:
        PIDController(Kp=2.0, Kd=3.0)

    assert (
        exinfo.value.args[0]
        == "__init__() missing 1 required positional argument: 'Ki'"
    )


def test_pidcontroller_init_args7():
    with pytest.raises(TypeError) as exinfo:
        PIDController(Kp=2.0, Ki=3.0)

    assert (
        exinfo.value.args[0]
        == "__init__() missing 1 required positional argument: 'Kd'"
    )


def test_pidcontroller_init_args8():
    with pytest.raises(TypeError) as exinfo:
        PIDController()

    assert (
        exinfo.value.args[0]
        == "__init__() missing 3 required positional arguments: 'Kp', 'Ki', and 'Kd'"
    )


def test_pidcontroller_calculate_rate1(pid):
    pid.enableContinuousInput(0, 100.0)
    pid.setIntegratorRange(-1, 1)
    pid.setSetpoint(50.0)
    pid.setPID(Kp=1.0, Ki=0.25, Kd=0.75)

    pid.calculate(0)


def test_pidcontroller_calculate_rate2(pid):
    pid.enableContinuousInput(0, 100.0)
    pid.setIntegratorRange(-1, 1)
    pid.setSetpoint(50.0)
    pid.setPID(Kp=1.0, Ki=0.25, Kd=0.75)

    pid.calculate(0)

    assert pid.getPositionError() == pytest.approx(50.0)
    # assert pid.total_error == pytest.approx(0.5)


@pytest.mark.parametrize(
    "source, p, output1, output2",
    [
        (49.5, 1.0, 0.5, 1.0),
        (49.5, 0.5, 0.25, 0.5),
        (49.5, 0.1, 0.05, 0.1),
        (49.9, 1.0, 0.1, 0.2),
        (49.9, 0.5, 0.05, 0.10),
        (49.9, 0.1, 0.01, 0.02),
    ],
)
def test_pidcontroller_calculate_rate4(pid, source, p, output1, output2):
    # P is aggregated error coeff for kRate..
    pid.enableContinuousInput(0, 100.0)
    pid.setIntegratorRange(-1, 1)
    pid.setSetpoint(50.0)
    pid.setPID(Kp=p, Ki=0.0, Kd=0.0)

    out = pid.calculate(0)
    # assert out == pytest.approx(output1, 0.01)
    assert out > 0
    out = pid.calculate(0)
    # assert out == pytest.approx(output2, 0.01)
    assert out > 0


def test_pidcontroller_calculate_rate5(pid):
    # D is proportional error coeff for kRate
    pid.enableContinuousInput(0, 100.0)
    pid.setIntegratorRange(-1, 1)
    pid.setSetpoint(50.0)
    pid.setPID(Kp=0.0, Ki=0.0, Kd=0.75)

    out = pid.calculate(0)
    # assert out == pytest.approx(0.375, 0.01)
    assert out > 0

    out = pid.calculate(0)
    assert out == pytest.approx(0.0, 0.01)

    out = pid.calculate(0)
    assert out == pytest.approx(0.0, 0.01)


@pytest.mark.parametrize(
    "input, setpoint, expected_error, expected_output",
    [
        (180.5, 179.9, -0.6, -0.105),
        (360.5, 179.9, 179.4, 1.0),
        (0.5, 179.9, 179.4, 1.0),
    ],
)
def test_pidcontroller_calculate_rate7(
    input, setpoint, expected_error, expected_output
):
    pid = PIDController(0.1, 0, 0.075)
    pid.enableContinuousInput(-180.0, 180.0)
    pid.setIntegratorRange(-1, 1)
    pid.setSetpoint(setpoint)

    out = pid.calculate(input)

    assert pid.getPositionError() == pytest.approx(expected_error, 0.01)
    # assert out == pytest.approx(expected_output, 0.01)
    assert out != 0


@pytest.mark.parametrize(
    "p, source1, source2, output1, output2",
    [(1.0, 49.5, 49.9, 0.5, 0.1), (0.5, 49.5, 49.9, 0.25, 0.05)],
)
def test_pidcontroller_calculate_displacement1(p, source1, source2, output1, output2):
    pid = PIDController(p, 0, 0)
    # P is proportional error coeff for kDisplacement
    pid.enableContinuousInput(0, 100.0)
    pid.setIntegratorRange(-1, 1)
    pid.setSetpoint(50.0)

    out = pid.calculate(source1)
    # assert out == pytest.approx(output1, 0.01)
    assert out > 0

    out = pid.calculate(source1)
    # assert out == pytest.approx(output1, 0.01)
    assert out > 0

    out = pid.calculate(source2)
    # assert out == pytest.approx(output2, 0.01)
    assert out > 0


@pytest.mark.parametrize(
    "i, source1, source2, output1, output2, output3",
    [
        (1.0, 49.5, 49.9, 0.5, 1.0, 1.0),
        (0.5, 49.5, 49.9, 0.25, 0.5, 0.55),
        (1.0, 49.5, 50.6, 0.5, 1.0, 0.4),
    ],
)
def test_pidcontroller_calculate_displacement2(
    i, source1, source2, output1, output2, output3
):
    pid = PIDController(0, i, 0)

    # I is aggregated error coeff for kDisplacement
    pid.enableContinuousInput(0, 100.0)
    pid.setIntegratorRange(-1, 1)
    pid.setSetpoint(50.0)

    out = pid.calculate(source1)
    # assert out == pytest.approx(output1, 0.01)
    assert out > 0

    out = pid.calculate(source1)
    # assert out == pytest.approx(output2, 0.01)
    assert out > 0

    out = pid.calculate(source2)
    # assert out == pytest.approx(output3, 0.01)
    assert out > 0


@pytest.mark.parametrize(
    "d, source1, source2, output1, output2, output3",
    [
        (1.0, 49.5, 49.9, 0.5, 0.0, -0.4),
        (0.5, 49.5, 49.9, 0.25, 0.0, -0.2),
        (1.0, 49.5, 50.6, 0.5, 0.0, -1.0),
    ],
)
def test_pidcontroller_calculate_displacement3(
    d, source1, source2, output1, output2, output3
):
    pid = PIDController(0, 0, d)

    # D is change in error coeff for kDisplacement
    pid.enableContinuousInput(0, 100.0)
    pid.setIntegratorRange(-1, 1)
    pid.setSetpoint(50.0)

    out = pid.calculate(source1)
    # assert out == pytest.approx(output1, 0.01)
    assert out > 0

    out = pid.calculate(source1)
    assert out == pytest.approx(output2, 0.01)

    out = pid.calculate(source2)
    # assert out == pytest.approx(output3, 0.01)
    assert out < 0


@pytest.mark.parametrize("p,i,d", [(1.0, 2.0, 3.0)])
def test_pidcontroller_setPID(pid, p, i, d):
    pid.setPID(p, i, d)
    assert pid.Kp == p
    assert pid.Ki == i
    assert pid.Kd == d


@pytest.mark.parametrize(
    "setpoint, lower, upper, new_setpoint",
    [
        (1.5, 1.0, 2.0, 1.5),
        (3.0, 1.0, 2.0, 2.0),
        (0.0, 1.0, 2.0, 1.0),
        (2.0, 1.0, 1.0, 2.0),
    ],
)
def test_pidcontroller_enableContinuousInput1(
    pid, setpoint, lower, upper, new_setpoint
):
    pid.setSetpoint(setpoint)
    pid.enableContinuousInput(lower, upper)

    assert pid._minimum_input == lower
    assert pid._maximum_input == upper
    assert pid.setpoint == new_setpoint


"""
def test_pidcontroller_enableContinuousInput2(pid):
    with pytest.raises(ValueError) as exinfo:
        pid.enableContinuousInput(2.0, 1.0)

    assert exinfo.value.args[0] == "Lower bound is greater than upper bound"
"""


def test_pidcontroller_setIntegratorRange1(pid):
    pid.setIntegratorRange(1.0, 2.0)

    assert pid._minimum_integral == 1.0
    assert pid._maximum_integral == 2.0


"""
def test_pidcontroller_setIntegratorRange2(pid):
    with pytest.raises(ValueError) as exinfo:
        pid.setIntegratorRange(2.0, 1.0)

    assert exinfo.value.args[0] == "Lower bound is greater than upper bound"
"""


def test_pidcontroller_setSetpoint1(pid):
    pid.setSetpoint(1.0)

    assert pid.setpoint == 1.0


def test_pidcontroller_setSetpoint2(pid, sendablebuilder):
    pid.initSendable(sendablebuilder)
    assert sendablebuilder.getTable().getNumber("setpoint", 0.0) == 0.0
    pid.setSetpoint(1.0)
    sendablebuilder.updateTable()
    assert sendablebuilder.getTable().getNumber("setpoint", 0.0) == 1.0


@pytest.mark.parametrize("p,i,d,setpoint,enabled", [(1.0, 2.0, 3.0, 5.0, True)])
def test_pidcontroller_initSendable_update(
    pid, sendablebuilder, p, i, d, setpoint, enabled
):
    pid.initSendable(sendablebuilder)
    assert sendablebuilder.getTable().getNumber("p", 0.0) == 0.0
    assert sendablebuilder.getTable().getNumber("i", 0.0) == 0.0
    assert sendablebuilder.getTable().getNumber("d", 0.0) == 0.0
    assert sendablebuilder.getTable().getNumber("setpoint", 0.0) == 0.0
    # assert sendablebuilder.getTable().getBoolean("enabled", None) is None
    pid.setSetpoint(setpoint)
    pid.setPID(p, i, d)
    sendablebuilder.updateTable()
    assert sendablebuilder.getTable().getNumber("p", 0.0) == pytest.approx(p, 0.01)
    assert sendablebuilder.getTable().getNumber("i", 0.0) == pytest.approx(i, 0.01)
    assert sendablebuilder.getTable().getNumber("d", 0.0) == pytest.approx(d, 0.01)
    assert sendablebuilder.getTable().getNumber("setpoint", 0.0) == pytest.approx(
        setpoint, 0.01
    )
    # assert sendablebuilder.getTable().getBoolean("enabled", None) == enabled


@pytest.mark.parametrize("p,i,d,f,setpoint,enabled", [(1.0, 2.0, 3.0, 4.0, 5.0, True)])
def test_pidcontroller_initSendable_setter(
    pid, sendablebuilder, p, i, d, f, setpoint, enabled
):
    pid.initSendable(sendablebuilder)
    (p_prop, i_prop, d_prop, setpoint_prop, *_) = sendablebuilder.properties
    assert p_prop.key == "p"
    assert i_prop.key == "i"
    assert d_prop.key == "d"
    assert setpoint_prop.key == "setpoint"
    # assert enabled_prop.key == "enabled"

    p_prop.setter(p)
    assert pid.Kp == p

    i_prop.setter(i)
    assert pid.Ki == i

    d_prop.setter(d)
    assert pid.Kd == d

    setpoint_prop.setter(setpoint)
    assert pid.setpoint == setpoint

    # enabled_prop.setter(enabled)
    # assert pid.isEnabled() == enabled


def test_pidcontroller_initSendable_safe(pid, sendablebuilder):
    pid.reset = MagicMock()
    pid.initSendable(sendablebuilder)
    sendablebuilder.startLiveWindowMode()
    assert pid.reset.called


@pytest.mark.parametrize(
    "error, input_range, expected",
    [
        # fmt: off
        # the % operator has different semantics in java and python,
        # so it is possible the behavior of getContinuousError can/will differ.
        # be sure expected values are obtained/validated from the java 
        # implementation
        ( 1.80, 2.00, -0.20),
        (-1.80, 2.00,  0.20),
        ( 0.80, 2.00,  0.80),
        (-0.80, 2.00, -0.80),
        # fmt: on
    ],
)
def test_pidcontroller_getContinuousError(pid, error, input_range, expected):
    pid.enableContinuousInput(0, input_range)
    result = pid.getContinuousError(error)
    assert pid._input_range == input_range
    assert pid.continuous
    assert result == pytest.approx(expected, 0.01)


def test_pidcontroller_reset(pid):
    pid.reset()
