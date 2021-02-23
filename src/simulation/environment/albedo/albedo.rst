
Executive Summary
-----------------
This document describes how albedo is modeled in the Basilisk software. The purpose of this module is to calculate
albedo value at the given instrument locations.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg variable names are set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - spacecraftStateInMsg
      - :ref:`SCStatesMsgPayload`
      - input message with thruster commands
    * - sunPositionInMsg
      - :ref:`SpicePlanetStateMsgPayload`
      - Sun input message
    * - planetInMsgs
      - :ref:`SpicePlanetStateMsgPayload`
      - vector of planet input messages are set by using either the ``addPlanetandAlbedoAverageModel()`` method and
        ``addPlanetandAlbedoDataModel()`` method
    * - albOutMsgs
      - :ref:`AlbedoMsgPayload`
      - vector of albedo output message names.  The order matches the order with which instruments are added.



Detailed Module Description
---------------------------
The albedo module is responsible for determining whether or not an incremental area on a planet is sunlit and within
the field of view of an instrument and if so, what is the albedo value at the instrument's location.
The module uses the configuration of the instruments, and states of the sun, spacecraft and planets. It calculates
the albedo value at the instrument locations as ratio [-] and albedo flux :math:`[W/m^2]`. Albedo ratio of :math:`0.0` represents
no albedo, and :math:`1.0` represents an equal flux with solar flux at the location.

To determine the states of the bodies in question, messages are passed into the code. For the spacecraft, Cartesian
vectors provide the position and velocity of its center of mass. For the sun and planets their ephemeris messages are
read in. If given multiple instruments, the code iterates through the sensor list and creates an array consisting of
albedo values for each instrument. If given multiple planets, the code iterates through the planet list but sum all
the values at the end and creates only one value for each instrument.

Determining States
~~~~~~~~~~~~~~~~~~
The initial step in the module is to obtain the albedo model, then compute the albedo. In computing albedo,
the celestial body state data are obtained and transformed into usable terms.
The relationships shown below remove the dependency on relating position to the inertial frame :math:`N` and instead
utilize the spacecraft body :math:`B`, instrument body :math:`I`, sun :math:`S`, planet :math:`P`, and
incremental area :math:`dA` on the planet.

.. math:: \mathbf{r}_{BP} = \mathbf{r}_{BN} - \mathbf{r}_{PN}
    :label: eq:albedo:1

.. math:: \mathbf{r}_{IB} = \mathbf{r}_{IN} - \mathbf{r}_{BN}
    :label: eq:albedo:2

.. math:: \mathbf{r}_{IP} = \mathbf{r}_{IB} + \mathbf{r}_{BP}
    :label: eq:albedo:3

.. math:: \mathbf{r}_{SP} = \mathbf{r}_{SN} - \mathbf{r}_{PN}
    :label: eq:albedo:4

.. math:: \mathbf{r}_{IdA} = \mathbf{r}_{IP} - \mathbf{r}_{dAP}
    :label: eq:albedo:5

.. math:: \mathbf{r}_{SdA} = \mathbf{r}_{SP} - \mathbf{r}_{dAP}
    :label: eq:albedo:6

The previous two equations provide the sun's and instrument's position with respect to the incremental area using
Eq. :eq:`eq:albedo:1` - :eq:`eq:albedo:4` and :math:`\mathbf{r}_{dAP}`, which is transformed from latitude and longitude of the
grid points.

Sunlit Field of View Area
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
In determining the illuminated area within the instrument's fov, :math:`f_1`, :math:`f_2` and :math:`f_3`
are computed as shown below,

.. math:: f_1 = \frac{\mathbf{r}_{dAP}}{| \mathbf{r}_{dAP}|} \cdot \frac{\mathbf{r}_{SdA}}{| \mathbf{r}_{SdA}|}
    :label: eq:albedo:7

.. math:: f_2 = \frac{\mathbf{r}_{dAP}}{| \mathbf{r}_{dAP}|} \cdot \frac{\mathbf{r}_{IdA}}{| \mathbf{r}_{IdA}|}
    :label: eq:albedo:8

.. math:: f_3 = \hat{n}_N \cdot \frac{-\mathbf{r}_{IdA}}{| \mathbf{r}_{IdA}|}
    :label: eq:albedo:9

Here :math:`\hat{n}_N` indicates the unit normal vector of the instrument in inertial frame. :math:`f_1 > 0` presents
the sunlit :math:`f_2 > 0` presents the instrument's maximum fov, :math:`f_3 > \cos(fov)` presents the instrument's
specified fov.

Albedo module needs three variables related to instrument's configuration which are instrument's misalignment vector
with respect to spacecraft's body frame (:math:`r_{{IB}_B}`), unit normal vector of the instrument in spacecraft body
frame (:math:`\hat{n}_B`), and instrument's field of view half angle in radian (:math:`fov`). These variables can be
added to the module using ``addInstrumentConfig()`` method. First term for the method is the instrument name. The
rest of the terms can be set using the ``instConfig_t`` class or variable by variable respectively as: :math:`fov`,
:math:`\hat{n}_B`, and :math:`r_{{IB}_B}`.

In the module, for planets that have polar radius, :math:`RP_{planet}` and equatorial radius, :math:`REQ_{planet}` defined,
authalic radius is calculated. By doing this, the sphere is having the same surface area with the reference ellipsoid.
If the polar radius is not defined, module uses the equatorial radius.

Albedo Value
~~~~~~~~~~~~
Albedo flux ratio can be calculated as,

.. math:: \text{albedoAtInstrument} = ALB \frac{f_1 \cdot f_2 \cdot f_3 \cdot d_{Area}}{\pi \cdot |\mathbf{r}_{IdA}|^2}
    :label: eq:albedo:10

where :math:`d_{Area}` is the area of the incremental area, :math:`ALB` is the albedo coefficient. There are albedo models
based on an average albedo value and albedo data. The existing data files are placed under
``Basilisk/supportData/AlbedoData`` as ``.csv`` file format consisting :math:`ALB` matrix. The number of rows represent the
:math:`numLat`, number of latitude (between -90 to 90 deg) and columns represent the :math:`numLon`, number of longitude
(between -180 to 180 deg).

The Earth's albedo data is obtained from `CERES instrument <https://ceres.larc.nasa.gov/data/>`__ as .nd format and
converted to .csv format for consistency with 1x1, 5x5, and 10x10 degree resolutions under clear-sky and all-sky
conditions.

The Mars' albedo data is obtained from `TES instrument <http://www.mars.asu.edu/data/tes_albedo/>`__ as VICAR format
and converted to .csv format for consistency with 1x1, 5x5, and 10x10 degree resolutions.

``shadowFactorAtdA`` is optional to be calculated with eclipseCase being True or can be assigned
directly by the user with eclipseCase False. It is used as a multiplication term in Eq. :eq:`eq:albedo:10`, if defined.
Therefore, when using albedo output on an instrument, it should be used after the shadow factor multiplication of the
instrument, if exists.

A limit can be set in order not to compute the albedo for planets too far by :math:`altitudeRateLimit` which is the
limit for the rate of the instrument's altitude to the planet's radius.

Module Assumptions and Limitations
----------------------------------

- **Albedo Model:** The albedo models based on average value or specified data can be used.
- **Planet Shape:** The module uses approximated authalic sphere which has the same surface area with the reference ellipsoid.
- **Planet Radius:** The module have a list of planets with specified radius.

User Guide
----------
This section outlines the steps needed to add Albedo module to a sim. First, the albedo module should be imported:

.. code-block:: python

      from Basilisk.simulation import albedo
      albModule = albedo.Albedo()
      albModule.ModelTag = "Albedo_module"

The instruments' configuration must be added by using,

.. code-block:: python

      instConfig = albedo.instConfig_t()
      instConfig.fov
      instConfig.nHat_B
      instConfig.r_IB_B
      albModule.addInstrumentConfig(instConfig)

or by using,

.. code-block:: python

      albModule.addInstrumentConfig(fov, nHat_B, r_IB_B)

In the first case, if the variables are not defined for some reason and they are empty; then, default values are going
to be used as :math:`fov = 90.` deg, :math:`\hat{n}_B = [ 1.0, 0.0, 0.0 ]`, :math:`r_{{IB}_B} = [ 0.0, 0.0, 0.0 ]`.
The default values can be defined by the user as well. Both functions for the instrument configuration has the ability
to do a sanity check for :math:`fov` being positive and :math:`\hat{n}_B` not having all zero elements.
Also, :math:`\hat{n}_B` is always normalized. Then, the planet and albedo model function must be added.

There are three options based on the albedo model to be used.  In all cases the planet input message is
provided as an argument and the method makes the albedo modue subscribe to this message.
For ``ALBEDO_AVG`` case,

.. code-block:: python

      albModule.addPlanetandAlbedoAverageModel(planetMsg)

where albedo average value is calculated automatically based on the given planet, and

.. code-block:: python

      albModule.addPlanetandAlbedoAverageModel(planetMsg, ALB_avg, numLat, numLon)

where the user can set the albedo average value. Number of latitude/longitude can also be specified or set to a negative
value to let default values being used instead (``defaultNumLat = 180`` and ``defaultNumLon = 360``). The default values can
be changed by the user as well.
For ``ALBEDO_DATA`` case,

.. code-block:: python

      albModule.addPlanetandAlbedoDataModel(planetMsg, dataPath, fileName)

where the user can define the data path and file name for the albedo data to be used.
The model can  be added to a task like other simModels.

.. code-block:: python

      unitTestSim.AddModelToTask(simTaskName, albModule)
