Installation
------------

For local development on a computer with an internet connection:

.. code-block:: bash

   pip install wpilib-controller

To install onto an offline roboRIO:

.. code-block:: bash

   # with an internet connection
   pip download wpilib-controller -d pip_cache
   # whilst connected to your robot
   robotpy-installer install-pip wpilib-controller
