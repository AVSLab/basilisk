Executive Summary
-----------------
This module is used to generate noisy ephemeris data, similar to what :ref:`simpleNav` does for spacecraft states. These
noisy states can either serve as a stand-in for a filter that estimates the body's ephemeris or as measurements input
into such a filter. This module is most useful for small bodies like comets or asteroids where large uncertainty in the
body's ephemeris is present. This module is not recommended for larger bodies like the Earth or the sun.

The noise present in the planetNav module is designed to mimic the error signals that will be observed in the real
navigation system. The true ”noise” present in an orbit determination nav system is always a combination of bias,
white noise, and brown noise (or random walk). In order to provide this, a second-order Gauss-Markov process model was
added to the simulation utilities that allows the user to configure a random walk process.

Model Functions
---------------
This module allows the user to set the bounds and the standard deviations for:

- The body's inertial position
- The body's inertial velocity
- The body's attitude with respect to the inertial frame
- The body's angular velocity with respect to the inertial frame expressed in body-frame components

The user can set the noise levels of each of these parameters independently.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  
The module msg connection is set by the user from python.  
The msg type contains a link to the message structure definition, while the description 
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - ephemerisInMsg
      - :ref:`ephemerisMsgPayload`
      - Planet ephemeris input msg
    * - ephemerisOutMsg
      - :ref:`ephemerisMsgPayload`
      - Planet ephemeris output msg

User Guide
----------
To create a planetNav module, first instantiate a planetNav object. Then, set the walkBounds and standard deviation of
each state. Afterwards, set the ``crossTrans`` and ``crossAtt`` boleans based on whether or not position error should
depend on velocity or attitude error should depend on angular velocity.

.. code-block:: python

    planetNavigation = planetNav.PlanetNav()
    planetNavigation.ModelTag = "planetNavigation"
    planetNavigation.walkBounds = errorBounds
    planetNavigation.PMatrix = pMatrix
    planetNavigation.crossTrans = True
    planetNavigation.crossAtt = False
    scSim.AddModelToTask(simTaskName, planetNavigation)