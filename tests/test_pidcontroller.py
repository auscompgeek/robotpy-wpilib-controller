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
    source = MagicMock()
    source.return_value = 0
    _pid = PIDController(Kp=1.0, Ki=0.25, Kd=0.75, measurement_source=source)
    return _pid


def test_pidcontroller_init_args1():
    source = lambda: 77.0
    pid = PIDController(
        1.0, 2.0, 3.0, feedforward=lambda: 4.0, measurement_source=source, period=5.0
    )

    assert pid.Kp == pytest.approx(1.0, 0.01)
    assert pid.Ki == pytest.approx(2.0, 0.01)
    assert pid.Kd == pytest.approx(3.0, 0.01)
    assert pid.period == pytest.approx(5.0, 0.01)


def test_pidcontroller_init_args2():
    source = lambda: 77.0
    pid = PIDController(1.0, 2.0, 3.0, measurement_source=source, period=5.0)

    assert pid.Kp == pytest.approx(1.0, 0.01)
    assert pid.Ki == pytest.approx(2.0, 0.01)
    assert pid.Kd == pytest.approx(3.0, 0.01)
    assert pid.period == pytest.approx(5.0, 0.01)


def test_pidcontroller_init_args3():
    source = lambda: 77.0
    pid = PIDController(1.0, 2.0, 3.0, measurement_source=source)

    assert pid.Kp == pytest.approx(1.0, 0.01)
    assert pid.Ki == pytest.approx(2.0, 0.01)
    assert pid.Kd == pytest.approx(3.0, 0.01)
    assert pid.period == pytest.approx(0.05, 0.01)


def test_pidcontroller_init_args4():
    source = lambda: 77.0
    pid = PIDController(
        1.0, 2.0, 3.0, feedforward=lambda: 4.0, measurement_source=source
    )

    assert pid.Kp == pytest.approx(1.0, 0.01)
    assert pid.Ki == pytest.approx(2.0, 0.01)
    assert pid.Kd == pytest.approx(3.0, 0.01)
    assert pid.feedforward() == pytest.approx(4.0, 0.01)
    assert pid.period == pytest.approx(0.05, 0.01)


def test_pidcontroller_init_args0():
    source = lambda: 77.0
    pid = PIDController(
        period=5.0,
        Ki=2.0,
        Kp=1.0,
        Kd=3.0,
        feedforward=lambda: 4.0,
        measurement_source=source,
    )

    assert pid.Kp == pytest.approx(1.0, 0.01)
    assert pid.Ki == pytest.approx(2.0, 0.01)
    assert pid.Kd == pytest.approx(3.0, 0.01)
    assert pid.period == pytest.approx(5.0, 0.01)


def test_pidcontroller_init_args5():
    source = lambda: 77.0
    with pytest.raises(TypeError) as exinfo:
        PIDController(
            Ki=2.0, Kd=3.0, feedforward=lambda: 4.0, measurement_source=source
        )

    assert (
        exinfo.value.args[0]
        == "__init__() missing 1 required positional argument: 'Kp'"
    )


def test_pidcontroller_init_args6():
    source = lambda: 77.0
    with pytest.raises(TypeError) as exinfo:
        PIDController(
            Kp=2.0, Kd=3.0, feedforward=lambda: 4.0, measurement_source=source
        )

    assert (
        exinfo.value.args[0]
        == "__init__() missing 1 required positional argument: 'Ki'"
    )


def test_pidcontroller_init_args7():
    source = lambda: 77.0
    with pytest.raises(TypeError) as exinfo:
        PIDController(
            Kp=2.0, Ki=3.0, feedforward=lambda: 4.0, measurement_source=source
        )

    assert (
        exinfo.value.args[0]
        == "__init__() missing 1 required positional argument: 'Kd'"
    )


def test_pidcontroller_init_args8():
    source = lambda: 77.0
    with pytest.raises(TypeError) as exinfo:
        PIDController(feedforward=lambda: 4.0, measurement_source=source)

    assert (
        exinfo.value.args[0]
        == "__init__() missing 3 required positional arguments: 'Kp', 'Ki', and 'Kd'"
    )


def test_pidcontroller_init_args10():
    source = lambda: 77.0
    pid = PIDController(
        Ki="2.0", Kp=1.0, Kd=3.0, feedforward=lambda: 4.0, measurement_source=source
    )

    assert pid.Kp == pytest.approx(1.0, 0.01)
    # eh?
    assert pid.Ki == "2.0"
    assert pid.Kd == pytest.approx(3.0, 0.01)
    assert pid.period == pytest.approx(0.05, 0.01)


def test_pidcontroller_calculate_rate1(pid):
    pid.setInputRange(0, 100.0)
    pid.setOutputRange(-1, 1)
    pid.setReference(50.0)
    pid.setPID(Kp=1.0, Ki=0.25, Kd=0.75)

    pid.update()


def test_pidcontroller_calculate_rate2(pid):
    pid.setInputRange(0, 100.0)
    pid.setOutputRange(-1, 1)
    pid.setReference(50.0)
    pid.setPID(Kp=1.0, Ki=0.25, Kd=0.75)

    pid.update()

    assert pid.prev_error == pytest.approx(50.0)
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
    pid.setInputRange(0, 100.0)
    pid.setOutputRange(-1, 1)
    pid.setReference(50.0)
    pid.setPID(Kp=p, Ki=0.0, Kd=0.0)

    out = pid.update()
    # assert out == pytest.approx(output1, 0.01)
    assert out > 0
    out = pid.update()
    # assert out == pytest.approx(output2, 0.01)
    assert out > 0


def test_pidcontroller_calculate_rate5(pid):
    # D is proportional error coeff for kRate
    pid.setInputRange(0, 100.0)
    pid.setOutputRange(-1, 1)
    pid.setReference(50.0)
    pid.setPID(Kp=0.0, Ki=0.0, Kd=0.75)

    out = pid.update()
    # assert out == pytest.approx(0.375, 0.01)
    assert out > 0

    out = pid.update()
    assert out == pytest.approx(0.0, 0.01)

    out = pid.update()
    assert out == pytest.approx(0.0, 0.01)


def test_pidcontroller_calculate_rate6():
    source = MagicMock()
    source.return_value = 49.5
    pid = PIDController(0, 0, 0, feedforward=lambda: 0.01, measurement_source=source)

    # F is coeff for some feed forward calc for kRate
    pid.setInputRange(0, 100.0)
    pid.setOutputRange(-1, 1)
    pid.setReference(50.0)

    out = pid.update()
    # assert out == pytest.approx(0.5, 0.01)
    assert out > 0

    out = pid.update()
    # assert out == pytest.approx(0.5, 0.01)
    assert out > 0

    source.return_value = 49.9
    out = pid.update()
    # assert out == pytest.approx(0.5, 0.01)
    assert out > 0


@pytest.mark.parametrize(
    "continuous, input, setpoint, expected_error, expected_output",
    [
        (False, 180.5, 179.9, -0.6, -0.105),
        (False, 360.5, 179.9, -180.6, -1.0),
        (False, 0.5, 179.9, 179.4, 1.0),
        (True, 180.5, 179.9, -0.6, -0.105),
        (True, 360.5, 179.9, 179.4, 1.0),
        (True, 0.5, 179.9, 179.4, 1.0),
    ],
)
def test_pidcontroller_calculate_rate7(
    continuous, input, setpoint, expected_error, expected_output
):
    pid = PIDController(0.1, 0, 0.075, measurement_source=lambda: input)
    pid.setInputRange(-180.0, 180.0)
    pid.setContinuous(continuous)
    pid.setOutputRange(-1, 1)
    pid.setReference(setpoint)

    out = pid.update()

    assert pid.prev_error == pytest.approx(expected_error, 0.01)
    # assert out == pytest.approx(expected_output, 0.01)
    assert out != 0


@pytest.mark.parametrize(
    "p, source1, source2, output1, output2",
    [(1.0, 49.5, 49.9, 0.5, 0.1), (0.5, 49.5, 49.9, 0.25, 0.05)],
)
def test_pidcontroller_calculate_displacement1(p, source1, source2, output1, output2):
    source = MagicMock()
    source.return_value = source1
    pid = PIDController(p, 0, 0, measurement_source=source)
    # P is proportional error coeff for kDisplacement
    pid.setInputRange(0, 100.0)
    pid.setOutputRange(-1, 1)
    pid.setReference(50.0)

    out = pid.update()
    # assert out == pytest.approx(output1, 0.01)
    assert out > 0

    out = pid.update()
    # assert out == pytest.approx(output1, 0.01)
    assert out > 0

    source.return_value = source2
    out = pid.update()
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
    source = MagicMock()
    source.return_value = source1
    pid = PIDController(0, i, 0, measurement_source=source)

    # I is aggregated error coeff for kDisplacement
    pid.setInputRange(0, 100.0)
    pid.setOutputRange(-1, 1)
    pid.setReference(50.0)

    out = pid.update()
    # assert out == pytest.approx(output1, 0.01)
    assert out > 0

    out = pid.update()
    # assert out == pytest.approx(output2, 0.01)
    assert out > 0

    source.return_value = source2
    out = pid.update()
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
    source = MagicMock()
    source.return_value = source1
    pid = PIDController(0, 0, d, measurement_source=source)

    # D is change in error coeff for kDisplacement
    pid.setInputRange(0, 100.0)
    pid.setOutputRange(-1, 1)
    pid.setReference(50.0)

    out = pid.update()
    # assert out == pytest.approx(output1, 0.01)
    assert out > 0

    out = pid.update()
    assert out == pytest.approx(output2, 0.01)

    source.return_value = source2
    out = pid.update()
    # assert out == pytest.approx(output3, 0.01)
    assert out < 0


@pytest.mark.parametrize(
    "f, source1, source2, output1, output2, output3",
    [
        (1.0, 49.5, 49.9, 1.0, 0.0, 0.0),
        (0.5, 49.5, 49.9, 1.0, 0.0, 0.0),
        (1.0, 49.5, 50.6, 1.0, 0.0, 0.0),
    ],
)
def test_pidcontroller_calculate_displacement4(
    f, source1, source2, output1, output2, output3
):
    source = MagicMock()
    source.return_value = source1
    pid = PIDController(0, 0, 0, feedforward=lambda: f, measurement_source=source)

    # F is coeff for some feed forward calc for kDisplacement
    pid.setInputRange(0, 100.0)
    pid.setOutputRange(-1, 1)
    pid.setReference(50.0)

    out = pid.update()
    # assert out == pytest.approx(output1, 0.01)
    assert out > 0

    out = pid.update()
    # assert out == pytest.approx(output2, 0.01)

    source.return_value = source2
    out = pid.update()
    # assert out == pytest.approx(output3, 0.01)


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
def test_pidcontroller_setInputRange1(pid, setpoint, lower, upper, new_setpoint):
    pid.setReference(setpoint)
    pid.setInputRange(lower, upper)

    assert pid._minimum_input == lower
    assert pid._maximum_input == upper
    assert pid.reference == new_setpoint


"""
def test_pidcontroller_setInputRange2(pid):
    with pytest.raises(ValueError) as exinfo:
        pid.setInputRange(2.0, 1.0)

    assert exinfo.value.args[0] == "Lower bound is greater than upper bound"
"""


def test_pidcontroller_setOutputRange1(pid):
    pid.setOutputRange(1.0, 2.0)

    assert pid.minimum_output == 1.0
    assert pid.maximum_output == 2.0


"""
def test_pidcontroller_setOutputRange2(pid):
    with pytest.raises(ValueError) as exinfo:
        pid.setOutputRange(2.0, 1.0)

    assert exinfo.value.args[0] == "Lower bound is greater than upper bound"
"""


def test_pidcontroller_setReference1(pid):
    pid.setReference(1.0)

    assert pid.reference == 1.0


def test_pidcontroller_setReference2(pid, sendablebuilder):
    pid.initSendable(sendablebuilder)
    assert sendablebuilder.getTable().getNumber("setpoint", 0.0) == 0.0
    pid.setReference(1.0)
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
    assert sendablebuilder.getTable().getNumber("f", 0.0) == 0.0
    assert sendablebuilder.getTable().getNumber("setpoint", 0.0) == 0.0
    assert sendablebuilder.getTable().getBoolean("enabled", None) is None
    pid.setReference(setpoint)
    pid.setPID(p, i, d)
    sendablebuilder.updateTable()
    assert sendablebuilder.getTable().getNumber("p", 0.0) == pytest.approx(p, 0.01)
    assert sendablebuilder.getTable().getNumber("i", 0.0) == pytest.approx(i, 0.01)
    assert sendablebuilder.getTable().getNumber("d", 0.0) == pytest.approx(d, 0.01)
    assert sendablebuilder.getTable().getNumber("f", 0.0) == pytest.approx(0, 0.01)
    assert sendablebuilder.getTable().getNumber("setpoint", 0.0) == pytest.approx(
        setpoint, 0.01
    )
    # assert sendablebuilder.getTable().getBoolean("enabled", None) == enabled


@pytest.mark.parametrize("p,i,d,f,setpoint,enabled", [(1.0, 2.0, 3.0, 4.0, 5.0, True)])
def test_pidcontroller_initSendable_setter(
    pid, sendablebuilder, p, i, d, f, setpoint, enabled
):
    pid.initSendable(sendablebuilder)
    (p_prop, i_prop, d_prop, f_prop, setpoint_prop, *_) = sendablebuilder.properties
    assert p_prop.key == "p"
    assert i_prop.key == "i"
    assert d_prop.key == "d"
    assert f_prop.key == "f"
    assert setpoint_prop.key == "setpoint"
    # assert enabled_prop.key == "enabled"

    p_prop.setter(p)
    assert pid.Kp == p

    i_prop.setter(i)
    assert pid.Ki == i

    d_prop.setter(d)
    assert pid.Kd == d

    f_prop.setter(f)
    assert pid.feedforward() == f

    setpoint_prop.setter(setpoint)
    assert pid.reference == setpoint

    # enabled_prop.setter(enabled)
    # assert pid.isEnabled() == enabled


def test_pidcontroller_initSendable_safe(pid, sendablebuilder):
    pid.reset = MagicMock()
    pid.initSendable(sendablebuilder)
    sendablebuilder.startLiveWindowMode()
    assert pid.reset.called


@pytest.mark.parametrize(
    "error, input_range, expected, continuous",
    [
        # fmt: off
        # the % operator has different semantics in java and python,
        # so it is possible the behavior of getContinuousError can/will differ.
        # be sure expected values are obtained/validated from the java 
        # implementation
        ( 1.80, 2.00, -0.20, True),
        (-1.80, 2.00,  0.20, True),
        ( 0.80, 2.00,  0.80, True),
        (-0.80, 2.00, -0.80, True),
        ( 1.80, 2.00,  1.80, False),
        (-1.80, 2.00, -1.80, False),
        ( 0.80, 2.00,  0.80, False),
        (-0.80, 2.00, -0.80, False),
        # fmt: on
    ],
)
def test_pidcontroller_getContinuousError(
    pid, error, input_range, expected, continuous
):
    pid.setInputRange(0, input_range)
    pid.setContinuous(continuous)
    result = pid.getContinuousError(error)
    assert pid._input_range == input_range
    assert pid.continuous == continuous
    assert result == pytest.approx(expected, 0.01)


def test_pidcontroller_reset(pid):
    pid.reset()
