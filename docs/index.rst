RobotPy wpilib-controller documentation
=======================================

This is a backport of the 2020 WPILib new PIDController for RobotPy.

Please note that this has received some major changes since the 2019 season.

Note that if you are moving from the old WPILib PIDController, your PID
constants will need to change, as it did not consider the discretization period:

- divide your Ki gain by 0.05, and
- multiply your Kd gain by 0.05,
- where 0.05 is the original default period (use the period you used otherwise).

.. toctree::
   :maxdepth: 2
   :caption: Contents

   api
   install

Example usage
-------------

.. code-block:: python

   from wpilib_controller import PIDController

   controller = PIDController(1, 0, 0)
   # setInputRange and setContinuous are now a single method
   controller.enableContinuousInput(0, 360)

   controller.setSetpoint(180)

   # elsewhere...

   # assume gyro is a gyro object created prior
   output = controller.calculate(gyro.getAngle())
   # do something with the output, for example:
   motor.set(output)

Major differences
-----------------

- The PID gains are no longer time dependent.
- This PIDController expects a measurement as a parameter to ``calculate()``.
- This PIDController runs synchronously in your robot code (as opposed to having it run in a thread). You must call ``calculate()`` and use its return value.

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
