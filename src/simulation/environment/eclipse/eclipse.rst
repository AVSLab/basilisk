
Executive Summary
-----------------
The eclipse module is responsible for determining whether or not a spacecraft is experiencing a solar eclipse and
how much of it is occluded. The module works on a list of spacecraft state messages, as well as a list of planets.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg variable name is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.


.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - sunInMsg
      - :ref:`SpicePlanetStateMsgPayload`
      - sun ephemeris input message name
    * - planetInMsgs
      - :ref:`SpicePlanetStateMsgPayload`
      - A vector of planet incoming state message names ordered by the sequence in which planet are added to the module
    * - positionInMsgs
      - :ref:`SCStatesMsgPayload`
      - vector of msgs for each spacecraft position state for which to evaluate eclipse conditions
    * - eclipseOutMsgs
      - :ref:`EclipseMsgPayload`
      - vector of eclipse output msg names


Detailed Module Description
---------------------------
The eclipse module is responsible for determining whether or not a spacecraft is within the shadow of a solar eclipse and if so, how much. The module finds the states of the sun, spacecraft and planets of interest, allowing each body's position to be related and the construction of the conical shadow model. This provides the means for computing what percent of the spacecraft is illuminated where a shadow factor of 0.0 represents a total eclipse and 1.0 represents no eclipse.

To determine the states of the bodies in question, messages are passed into the code. For the spacecraft, Cartesian vectors provide the position and velocity of its center of mass. For the sun and planets their ephemeris messages are read in. The planets desired to be used in the module are specified through the python method ``addPlanetToModel()``, where the planet spice state message is the input.  If given multiple planets, the code iterates through the planet list and determines which is the closest to the spacecraft.  The figure below illustrates how the states are represented and will be identified in the mathematical model. Calculations in this model are taken from Montenbruck and Gill's text `Satellite Orbits Models, Methods and Applications <http://doi.org/10.1007/978-3-642-58351-3>`__.

.. _ConShad:
.. figure:: /../../src/simulation/environment/eclipse/_Documentation/Images/conical_shadow.svg
    :align: center

    Figure 1: Illustration of the Planet's Umbra and Penumbra Regions



Determining States
~~~~~~~~~~~~~~~~~~
The initial step in the eclipse module is to obtain the celestial bodies' state data and transform them into usable terms. The relationships shown below remove the dependency on relating position to the inertial frame :math:`N` and instead utilize the planet :math:`P`, spacecraft body :math:`B`, and helio :math:`H` frames.

.. math:: \mathbf{s}_{P/H} = \mathbf{r}_{N/H} - \mathbf{r}_{N/P}
    :label: eq:elipse:1

.. math:: \mathbf{r}_{B/H} = \mathbf{r}_{N/H} - \mathbf{r}_{N/B}
    :label: eq:elipse:2

.. math:: \mathbf{s}_{P/B} = \mathbf{r}_{N/B} - \mathbf{r}_{N/P}
    :label: eq:elipse:3

The previous three equations provide coordinates for the sun with respect to both the occulting planet and occulted spacecraft as well as the spacecraft's position with respect to the planet, respectively. The parameters on the right side of these equations come from the input state data where :math:`\mathbf{r}_{N/H}`, :math:`\mathbf{r}_{N/P}`, and :math:`\mathbf{r}_{N/B}` are the sun, planet, and spacecraft positions in the inertial frame.

This module supports the use of multiple occulting bodies, so it is important to analyze only the planet with the highest potential to cause an eclipse. Thus, the closest planet is determined by comparing the magnitude of each planet's distance to the spacecraft, :math:`|\mathbf{s}_{P/B}|`. Note that if the spacecraft is closer to the sun than the planet, i.e. :math:`|\mathbf{r}_{B/H}| < |\mathbf{s}_{P/H}|`, an eclipse is not possible and the shadow fraction is immediately set to 1.0.

Eclipse Conditions
~~~~~~~~~~~~~~~~~~

When analyzing the conical shadow model, there are critical distances and conical dimensions that must be considered. These parameters are determined by first knowing the planet's equatorial radius :math:`r_P`, which is used to solve for the angles of the shadow cones. Angles :math:`f_1` and :math:`f_2` are computed as shown below, where the subscript 1 relates to the cone of the penumbra and 2 relates to the umbra.

.. math:: f_1 = \arcsin\left(\frac{r_H + r_P}{| \mathbf{s}_{P/H}|} \right)
    :label: eq:elipse:7

.. math:: f_2 = \arcsin\left(\frac{r_H - r_P}{| \mathbf{s}_{P/H}|} \right)
    :label: eq:elipse:8

Here :math:`r_H` indicates the equatorial radius of the sun, which is 695000 km. Both the sun and planet radii must be input in terms of meters.

As shown by :ref:`ConShad`, the fundamental plane is perpendicular to the shadow axis and coincident with the spacecraft body. The distance between the plane-axis intersection and the center of the planet is given by :math:`s_0` as shown by Eq. :eq:`eq:elipse:9`.

.. math:: s_0 = \frac{-\mathbf{s}_{P/B} \cdot \mathbf{s}_{P/H}}{| \mathbf{s}_{P/H}|}
    :label: eq:elipse:9



This distance and the shadow cone angles can now be used to determine the distances, :math:`c_1` and :math:`c_2`, between the fundamental plane and the cones' vertices :math:`V_1` and :math:`V_2`. These are calculated as follows:

.. math:: c_1 = s_0 + \frac{r_P}{\sin(f_1)}
    :label: eq:elipse:10

.. math:: c_2 = s_0 - \frac{r_P}{\sin(f_2)}
    :label: eq:elipse:11

As shown in Eq. :eq:`eq:elipse:12` and :eq:`eq:elipse:13`, these are then used to find the radii, :math:`l_1` and :math:`l_2`, of the shadow cones in the fundamental plane.

.. math:: l_1 = c_1 \tan(f_1)
    :label: eq:elipse:12

.. math:: l_2 = c_2 \tan(f_2)
    :label: eq:elipse:13

Finding these parameters provides insight into the type of eclipse that the spacecraft is experiencing. To determine the type, it is useful to compare the cone radii to the distance between the spacecraft and the shadow axis, which is given by :math:`l`.

.. math:: l = \sqrt{|\mathbf{s}_{P/B}|^2 - s^2_0}
    :label: eq:elipse:14

Total and annular eclipses both require the spacecraft to be relatively close to the shadow axis, where :math:`|l|<| l_2|`. The difference between these two types is that the planet is closer to the spacecraft for a total eclipse (:math:`c_2 < 0`) than during an annular eclipse (:math:`c_2 > 0`). If the spacecraft is further from the shadow axis but still within a cone radius (:math:`|l|<| l_1|`), it is experiencing a partial eclipse.

Percent Shadow
~~~~~~~~~~~~~~

With the eclipse type determined, the shadow fraction can now be found. To find the shadow fraction, the apparent radii of the sun and planet and the apparent separation of both bodies are needed. These are given, respectively, by :math:`a`, :math:`b`, and :math:`c` in the equations below.

.. math:: a = \arcsin(\frac{r_H}{| \mathbf{r}_{B/H}|})
    :label: eq:elipse:15

.. math:: b = \arcsin(\frac{r_P}{| \mathbf{s}_{P/B}|})
    :label: eq:elipse:16

.. math:: c = \arccos(\frac{-\mathbf{s}_{P/B} \cdot \mathbf{r}_{B/H}}{| \mathbf{s}_{P/B}| | \mathbf{r}_{B/H}|})
    :label: eq:elipse:17

:ref:`FigDisk` below illustrates the overlapping disk model that represents the occultation, where the solid orange line indicates the sun and the dotted blue line indicates the planet.

.. _FigDisk:
.. figure:: /../../src/simulation/environment/eclipse/_Documentation/Images/diskModel.svg
    :align: center

    Figure 2: Occultation Disk Model

Total Eclipse (:math:`c < b-a`)
"""""""""""""""""""""""""""""""

This type assumes that the apparent radius of the planet is larger than that of the sun (:math:`b>a`). A total eclipse produces a total shadow, so the shadow fraction is 0.0.


Annular Eclipse ($c < a-b$)
"""""""""""""""""""""""""""

This type assumes the apparent radius of the sun is larger than that of the planet (:math:`a>b`). Use the equation for a circular area, :math:`A = \pi r^2`, to find the area of the sun and planet faces, replacing :math:`r` with the corresponding apparent radius. The shadow fraction is then just the ratio of the planet's area to the sun's area.
\begin{equation} \label{eq:18}
Shadow Fraction = \frac{A_P}{A_H}
\end{equation}

Partial Eclipse ($c < a+ b$)
""""""""""""""""""""""""""""

For a partial eclipse, the occulted area is given by Eq. :eq:`eq:elipse:19`.

.. math:: A = a^2 \arccos(\frac{x}{a}) + b^2 \arccos(\frac{c-x}{b}) - cy
    :label: eq:elipse:19

Parameters :math:`a`, :math:`b`, and :math:`c` are those calculated previously in Eq. :eq:`eq:elipse:15`, :eq:`eq:elipse:16`, and :eq:`eq:elipse:17`. The values :math:`x` and :math:`y` are given by the following equations.

.. math:: x = \frac{c^2 + a^2 - b^2}{2c}
    :label: eq:elipse:20

.. math:: y = \sqrt{a^2 - x^2}
    :label: eq:elipse:21

Like with the annular partial eclipse, the shadow factor for this type is the ratio between the occulted area and the sun's apparent area. This is given by the equation below.

.. math:: \text{Shadow Fraction} = 1 - \frac{A}{\pi a^2}
    :label: eq:elipse:22


Module Assumptions and Limitations
----------------------------------

- **Occultation Model:** Since the apparent radius of the sun is relatively small, the occultation can be modeled as
  overlapping disks.
- **No Eclipse:** If the spacecraft is closer to the sun than the planet, an eclipse is not possible.
- **Planets:** The allowed Spice planet names for use as occulting bodies are:

  - ``mercury``, ``venus``, ``earth``, ``mars``, ``mars barycenter``, ``jupiter barycenter``, ``saturn``,
    ``neptune``, ``uranus``, ``custom``
- **Custom Body:** This module allows for the use of a single custom body, defined by the name ``custom``. The name of
  the :ref:`SpicePlanetStateMsgPayload` must be set to ``custom``, and the variable ``rEqCustom`` must be set to
  some positive value in meters.
- **Sun and Planet States:** The data defining the sun and planet states is obtained through an external Spice package.
  Errors may be derived from this package but will be small.
- **Spacecraft States:** Spacecraft states must be input as Cartesian vectors. In the test, a conversion from orbital
  elements is performed.
- **Apparent Radii:** When determining the type of eclipse, assume that the apparent separation :math:`c \geq 0`.

  - **Total Eclipse** (:math:`c<b-a`): Assume the apparent radius of the planet is greater than that of
    the sun (:math:`b>a`).
  - **Annular Eclipse** (:math:`c<a-b`): Assume the apparent radius of the sun is greater than that of the
    planet (:math:`a>b`).

User Guide
----------

Setting The Spacecraft State Input Messages
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
The spacecrat state messages are read in by the ``eclipse`` module to determine where the spacecraft is relative to the sun and the planet(s).  The module can handle a list of input messages, however, these are not set directly.  Rather, use the method::

    addSpacecraftToModel(scObject.scStateOutMsg)

The spacecraft state message is of type :ref:`SCStatesMsgPayload`.

Setting The Planet State Input Messages
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
The planet state input message is of type :ref:`SpicePlanetStateMsgPayload`.  To add this as an input connection to ``eclipse``, use the method::

    addPlanetToModel(gravFactory.spiceObject.planetStateOutMsgs[0])

At least one planet must be specified.  If there are multiple planets, then the shadow factor is only computed relative to the closest planet.

Setting the Sun State Input Message (Optional)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
To connect to a sun state input message use::

    eclipseObject.sunInMsg.subscribeTo(sunMsg)

Eclipse Output Messages
~~~~~~~~~~~~~~~~~~~~~~~
The ``eclipse`` module will output a series of messages of type :ref:`EclipseMsgPayload` corresponding to the number of spacecraft provided.  The names of these output messages are auto-generated as follows::

    eclipseObject.eclipseOutMsgs[0]
    eclipseObject.eclipseOutMsgs[1]
    eclipseObject.eclipseOutMsgs[2]

where ``0`` indicates the first spacecraft shadow factor messages, etc.
