.. Warning::

   :beta:`simpleAntenna` is a beta module in an initial public release. This module might be
   subject to changes in future releases.

Executive Summary
-----------------
This document describes how the simpleAntenna module operates within the Basilisk astrodynamics simulation framework. The purpose of this module is to provide a simplified representation of an antenna for spacecraft-to-spacecraft and spacecraft-to-ground communication. The module abstracts away complex antenna parameters, such as detailed radiation patterns, and replaces them with general assumptions that enable simulation of typical satellite communication systems. The simplified antenna model is based on a 2D Gaussian beam pattern (see `Figure 1`_), sidelobes are neglected (see also :ref:`module-assumptions`).

This module enables users to simulate an antenna mounted on a spacecraft or placed in a ground environment (Earth) by configuring a set of key parameters (see :ref:`detailed-module-description`). The module must be connected to either a spacecraft state message (for space-based antennas) or a ground state message (for ground-based antennas).

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.
The module msg connection is set by the user from python.
The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 15 20 30 40
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
      - Note
    * - scInStateMsg
      - :ref:`SCStatesMsgPayload`
      - spacecraft state
      - Required for space based antennas
    * - groundInStateMsg
      - :ref:`GroundStateMsgPayload`
      - ground state
      - Required for ground based antennas
    * - antennaInStateMsg
      - :ref:`antennaStateMsgPayload`
      - setting antenna to [off / Rx / Tx / RxTx]
      - optional, default is 'Off'
    * - sunInMsg
      - :ref:`SpicePlanetStateMsgPayload`
      - sun ephemeris data input message
      - optional, required for advanced sky noise temperature calculation (see :ref:`detailed-module-description`)
    * - planetInMsgs
      - :ref:`SpicePlanetStateMsgPayload`
      - vector of planet ephemeris data input messages
      - optional, required for advanced sky noise temperature calculation (see :ref:`detailed-module-description`)
    * - sunEclipseInMsg
      - :ref:`EclipseMsgPayload`
      - sun eclipse state input Message
      - optional, required for advanced sky noise temperature calculation (see :ref:`detailed-module-description`)
    * - antennaOutStateMsg
      - :ref:`AntennaLogMsgPayload`
      - output msg description
      - output message containing antenna state information

.. _detailed-module-description:

Detailed Module Description
---------------------------
The simpleAntenna module simulates a directional antenna for spacecraft communication systems. It provides a simplified representation of antenna behavior by abstracting away complex antenna properties, such as detailed radiation patterns and receiver noise modeling. Instead, the radiation pattern is approximated using a 2D Gaussian field response (see `Figure 1`_ and `MATLAB Gaussian Antenna Element <https://www.mathworks.com/help/phased/ref/phased.gaussianantennaelement-system-object.html>`__).

The module computes key antenna performance metrics including the Equivalent Isotropically Radiated Power (EIRP), the Gain-To-Noise Temperature (G/T), and the Noise Power. These outputs can be used to evaluate link budgets for spacecraft communication systems.

The following parameters must be configured by the user:

.. list-table:: Module Parameters
   :widths: 15 20 10 30 25
   :header-rows: 1

   * - Parameter
     - Symbol
     - Unit
     - Description
     - Valid range
   * - Antenna name
     - \-
     - \-
     - Name of the antenna
     - Max. 20 characters
   * - Frequency
     - :math:`f`
     - Hz
     - Operating frequency of the antenna
     - :math:`30\,\mathrm{MHz} < f < 60\,\mathrm{GHz}`
   * - Bandwidth
     - :math:`B`
     - Hz
     - Bandwidth of the antenna
     - :math:`B > 0\;\mathrm{Hz}`
   * - Directivity
     - :math:`D_\mathrm{dB}`
     - dB
     - Antenna directivity in :math:`\mathrm{dB}`
     - :math:`D_\mathrm{dB} > 9\,\mathrm{dB}`
   * - Beam cross section shape
     - :math:`k`
     - \-
     - Ratio between HPBW in azimuth :math:`\phi` and elevation :math:`\theta` direction (for a rotation symmetrical beam, :math:`k=1`)
     - :math:`0.5 < k < 5`
   * - Transmission power
     - :math:`P_\mathrm{Tx}`
     - W
     - Transmission power of the antenna (The power which is drawn from the spacecraft power system)
     - :math:`P_\mathrm{Tx} > 0\,\mathrm{W}`
   * - Reception power (optional)
     - :math:`P_\mathrm{Rx}`
     - W
     - Reception power of the antenna (The power which is drawn from the spacecraft power system) (default: 0W)
     - :math:`P_\mathrm{Rx} \geq 0\,\mathrm{W}`
   * - Equivalent noise temperature
     - :math:`T_\mathrm{E}`
     - K
     - Equivalent noise temperature of the receiver system
     - :math:`T_\mathrm{E} > 0\,\mathrm{K}`
   * - Antenna efficiency
     - :math:`\eta_\mathrm{r}`
     - \-
     - Antenna radiation efficiency
     - :math:`0 < \eta_\mathrm{r} \leq 1`
   * - Ambient temperature (optional)
     - :math:`T_\mathrm{Ambient}`
     - K
     - Ambient temperature of the antenna. If not set by the user, this value is calculated
     - :math:`T_\mathrm{Ambient} > 0\,\mathrm{K}`
   * - Use Haslam Map (optional)
     - :code:`useHaslamMap`
     - [bool]
     - Enable/Disable the use of the Haslam 408 MHz all-sky survey map for sky noise temperature calculation. If false a omnidirectional galactic noise temperature is used. (default: :code:`False`)
     - :code:`True` or :code:`False`
   * - Antenna position vector in body frame (optional)
     - :math:`\mathbf{r}_\mathrm{AB}^\mathrm{B}`
     - m
     - Position vector of the antenna with respect to the spacecraft body frame
     - default: :math:`\mathbf{r}_\mathrm{AB}^\mathrm{B} = [0,0,0]^\mathrm{T}` (see :ref:`Figure coord`)
   * - MRP from body to antenna coordinate frame (optional)
     - :math:`\boldsymbol{\sigma}_\mathrm{AB}`
     - \-
     - MRP representing the rotation from the spacecraft body frame :math:`\mathrm{B}` to the antenna coordinate frame :math:`\mathrm{A}`
     - default: :math:`\boldsymbol{\sigma}_\mathrm{AB} = [0,0,0]^\mathrm{T}` (see :ref:`Figure coord`)

.. _Figure coord:
.. figure:: /../../src/simulation/communication/simpleAntenna/_Documentation/Images/CoordinateFrame.svg
   :width: 25%
   :align: center
   :alt: Shematic antenna coordinate frame :math:`\mathrm{A}` definition

   Figure 1: Illustration of the antenna coordinate frame :math:`{A}`

The simpleAntenna module allows users to place an antenna either on a spacecraft or in a ground environment. The environment is automatically determined by which state message is connected (spacecraft state or ground state); exactly one must be linked.

The module supports the following communication configurations:

- Spacecraft <--> Spacecraft
- Spacecraft <--> Ground (Earth)

Ground <--> Ground communication is not yet supported (see :ref:`module-assumptions`).

The antenna can be in either state:
   - **Off** : no transmission or reception
   - **Tx** : transmission only
   - **Rx** : reception only
   - **RxTx** : transmission and reception

These states can either be set via the :code:`antennaInStateMsg` input message or by setting the state directly with the setter for this property.

Radiation Pattern Model
^^^^^^^^^^^^^^^^^^^^^^^

.. _Figure 1:
.. figure:: /../../src/simulation/communication/simpleAntenna/_Documentation/Images/DocumentationBSKsimpleAntenna.svg
   :width: 60%
   :align: center
   :alt: Shematic of SimpleAntenna beam pattern

   Figure 1: Illustration of simpleAntenna 2D Gaussian beam pattern

The antenna radiation pattern is modeled as a 2D Gaussian beam pattern (see also :ref:`module-assumptions`).

The antenna radiation pattern is modeled as a 2D Gaussian beam. The radiation pattern is defined by the following equation (linear, voltage pattern)\:

.. math::

      f(\theta,\phi) = f_{0} \cdot \exp\Bigg[
       -2 \cdot \ln(2) \Bigg(
           \Big( \frac{\theta}{\theta_{\mathrm{HPBW}}} \Big)^2
           +
           \Big( \frac{\phi}{\phi_{\mathrm{HPBW}}} \Big)^2
       \Bigg)
   \Bigg]

The power gain as a function of off-boresight angles is:

.. math::

   G(\theta,\phi) = G_{0} \cdot \exp\Bigg[
       -4 \cdot \ln(2) \Bigg(
           \Big( \frac{\theta}{\theta_{\mathrm{HPBW}}} \Big)^2
           +
           \Big( \frac{\phi}{\phi_{\mathrm{HPBW}}} \Big)^2
       \Bigg)
   \Bigg]

Where :math:`G_{0}` is the boresight power gain (see :ref:`Figure 1`).

.. math::

   k\,\mathrm{[-]} = \frac{\phi_\mathrm{HPBW}}{\theta_\mathrm{HPBW}}

Setting of the radiation pattern parameters
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To set the radiation-pattern specific parameters, the user can set the directivity :math:`D_\mathrm{dB}` and the beam cross-section shape :math:`k`. From these two parameters, the Half-Power BeamWidths (HPBW) in elevation :math:`\theta_\mathrm{HPBW}` and azimuth :math:`\phi_\mathrm{HPBW}` are calculated\:

The unit less directivity :math:`D` and the boresight power gain :math:`G_{0}` are calculated from :math:`D_\mathrm{dB}` as follows\:

.. math::

   D\,\mathrm{[-]} = 10^{D_\mathrm{dB}/10}

   G_{0}\,\mathrm{[-]} = D \cdot \eta_\mathrm{r}

:math:`\theta_\mathrm{HPBW}` and :math:`\phi_\mathrm{HPBW}` are then calculated as follows\:

.. math::

   \theta_\mathrm{HPBW}\,\mathrm{[rad]} = \sqrt{\frac{\ln(2) \cdot 16}{D \cdot k}}

   \phi_\mathrm{HPBW}\,\mathrm{[rad]} = \theta_\mathrm{HPBW} \cdot k

Calculation of the Equivalent Isotropically Radiated Power (EIRP) / :math:`P_\mathrm{EIRP}`
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The antenna metric for transmitting antennas, :math:`P_\mathrm{EIRP}` is calculated as shown below\:

.. math::

   P_\mathrm{EIRP}\,\mathrm{[dB]} = 10 \cdot \log_{10}(P_\mathrm{Tx}) + D_\mathrm{dB} + 10 \cdot \log_{10}(\eta_\mathrm{r})

- where :math:`P_\mathrm{Tx}` is the transmission power in [W], :math:`D_\mathrm{dB}` is the antenna directivity in [dB], and :math:`\eta_\mathrm{r}` is the antenna efficiency [-].

Calculation of :math:`\frac{G}{T_\mathrm{S}}`
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

:math:`\frac{G}{T_\mathrm{S}}` is a key performance metric for a receiving antenna, it is calculated as shown below\:

.. math::
   \frac{G}{T_\mathrm{S}}[dB/K] = D_\mathrm{dB} + 10 \cdot \log_{10}(\eta_\mathrm{r}) - 10 \cdot \log_{10}(T_\mathrm{S})

- where :math:`D_\mathrm{dB}` is the antenna directivity in [dB], :math:`\eta_\mathrm{r}` is the antenna efficiency [-], and :math:`T_\mathrm{S}` is the system noise temperature (antenna + receiver) in [K] (see also\: :ref:`calculation-of-noise-power`).

.. _calculation-of-noise-power:

Calculation of the noise-power at the antenna
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The noise power is the power generated by the noise sources at the antenna and receiver. This property is important for the calculation of the Signal-To-Noise ratio (SNR) at the receiver.

The noise power :math:`P_\mathrm{N}` at the antenna is calculated as\:

.. math::

   P_\mathrm{N}\,\mathrm{[W]} = k_\mathrm{B} \cdot T_\mathrm{S} \cdot B

- where :math:`k_\mathrm{B}` is the Boltzmann constant, :math:`T_\mathrm{S}` is the system noise temperature (antenna + receiver), and :math:`B` is the antenna bandwidth

Calculation of noise temperatures
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
- The system noise temperature :math:`T_\mathrm{S}` is computed as the sum of the Antenna Noise Temperature :math:`T_\mathrm{Ant}` and the Equivalent Noise Temperature of the receiver :math:`T_\mathrm{E}`

.. math::

   T_\mathrm{S} = T_\mathrm{Ant} + T_\mathrm{E}

The Equivalent Noise Temperature :math:`T_\mathrm{E}` is a user defined parameter representing the noise contribution of the receiver system.

The Antenna Noise Temperature :math:`T_\mathrm{Ant}` depends on the sky noise temperature :math:`T_\mathrm{Sky}` and the Environment Temperature :math:`T_\mathrm{Ambient}`.

.. math::

   T_\mathrm{Ant} = (1 - \eta_\mathrm{r}) \cdot T_\mathrm{Ambient} + \eta_\mathrm{r} \cdot T_\mathrm{Sky}

For space-based antennas\:
~~~~~~~~~~~~~~~~~~~~~~~~~~~

:math:`T_\mathrm{Sky}` depends on the antenna pointing direction and is computed as a weighted sum of the Galactic Noise Temperature :math:`T_\mathrm{Galaxy}` and the Noise Temperature of the celestial bodies within the antenna beam coverage, :math:`T_\mathrm{Planet}`.
The weights are given by the antenna beam coverage fractions :math:`C_\mathrm{Galaxy}` and :math:`C_\mathrm{Planet}` (see also :ref:`module-overlap-area-computation`). The antenna gain :math:`G(\theta, \phi)` is not considered at this stage (see also :ref:`module-assumptions`).

- For space-based antennas, :math:`T_\mathrm{Ambient}` can be set by the user (optional) if it is not set, a default value of :math:`T_\mathrm{Ambient} = 150.0K` is used.
- The caculation of :math:`T_\mathrm{Sky}` is described in :ref:`Calculation of T_sky <calculation-of-tsky>`.

For ground-based antennas\:
~~~~~~~~~~~~~~~~~~~~~~~~~~~

- Based on the International Standard Atmosphere (ISA) model, :math:`T_\mathrm{Ambient}` for ground-based antennas is calculated as a function of altitude :math:`h` (in [m])\:

.. math::

   T_\mathrm{Ambient}[K] =
      T_\mathrm{0} - \frac{dT}{dh} \cdot h

- where :math:`T_\mathrm{0} = 288.15 [K]` is the sea-level standard temperature according to ISA, and :math:`\frac{dT}{dh} = 0.0065 [K/m]` is the temperature lapse rate for an ISA atmosphere (Earth).

.. _calculation-of-tsky:

Calculation of :math:`T_\mathrm{sky}`
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For space-based antennas
~~~~~~~~~~~~~~~~~~~~~~~~~~
The sky noise temperature for a spacecraft-mounted antenna is calculated based on the antenna position and orientation with respect to the Earth, Sun and Moon (other celestial bodies are at the moment not incuded). If the setting :code:`useHaslamMap=True` is enabled, the galactic noise temperature is calculated using the Haslam 408 MHz all-sky survey map. if :code:`useHaslamMap=False`, a simple model with a frequency dependent, omnidirectional galactic noise temperature is used.

.. math::

   T_\mathrm{Sky} = T_\mathrm{Sun} \cdot C_\mathrm{Sun} + T_\mathrm{Earth} \cdot C_\mathrm{Earth} + T_\mathrm{Moon} \cdot C_\mathrm{Moon} + T_\mathrm{Galaxy} \cdot C_\mathrm{Galaxy}

where:

- :math:`T_\mathrm{Sun}`, :math:`T_\mathrm{Earth}`, :math:`T_\mathrm{Moon}` are the brightness temperatures of the Sun, Earth, and Moon, respectively, obtained from lookup tables as a function of frequency.
- :math:`T_\mathrm{Galaxy}` is the galactic background noise temperature (see below).
- :math:`C_\mathrm{Sun}`, :math:`C_\mathrm{Earth}`, :math:`C_\mathrm{Moon}`, :math:`C_\mathrm{Galaxy}` are the beam coverage fractions for each source (see :ref:`module-overlap-area-computation`).

The galactic coverage fraction is computed as:

.. math::

   C_\mathrm{Galaxy} = 1 - C_\mathrm{Sun} - C_\mathrm{Earth} - C_\mathrm{Moon}

**Galactic noise temperature calculation:**

The galactic noise temperature :math:`T_\mathrm{Galaxy}` depends on the ``useHaslamMap`` setting:

- If ``useHaslamMap=True``: The galactic noise temperature is calculated using the `Haslam 408 MHz all-sky survey map <https://lambda.gsfc.nasa.gov/product/foreground/fg_2014_haslam_408_info.html>`__. The brightness temperature at 408 MHz is obtained based on the antenna boresight direction and beam size, then scaled to the operating frequency using a spectral index of :math:`\beta = -2.7`:

  .. math::

     T_\mathrm{Galaxy} = T_\mathrm{408MHz} \cdot \left( \frac{f}{408\,\mathrm{MHz}} \right)^{\beta} + T_\mathrm{CMB}

- If ``useHaslamMap=False``: A simplified omnidirectional galactic noise model is used, where the brightness temperature is obtained from a lookup table as a function of frequency:

  .. math::

     T_\mathrm{Galaxy} = T_\mathrm{Galaxy,table}(f) + T_\mathrm{CMB}

where :math:`T_\mathrm{CMB} = 2.725\,\mathrm{K}` is the Cosmic Microwave Background temperature.

**Eclipse handling:**

If the ``sunEclipseInMsg`` is connected and the spacecraft is in full eclipse (shadow factor = 0), the sun coverage is set to zero:

.. math::

   C_\mathrm{Sun} = 0 \quad \text{(during eclipse)}

For ground-based antennas
~~~~~~~~~~~~~~~~~~~~~~~~~~~

- For ground based antennas, :math:`T_\mathrm{sky}` is always assumed to be :math:`T_\mathrm{sky} = 200K`.

.. _module-overlap-area-computation:

Overlap Area Computation
^^^^^^^^^^^^^^^^^^^^^^^^

In normalized beam space, the antenna beam is represented as a unit circle
with radius

.. math::

   R = 1

and the apparent planet disk is represented as a circle of radius :math:`r`
whose center lies at distance :math:`d` from the beam center.

Depending on the relative values of :math:`d`, :math:`R`, and :math:`r`,
three cases are distinguished.

No Overlap
~~~~~~~~~~

If the two circles do not intersect,

.. math::

   d \ge R + r

then the planet does not contribute to the antenna beam coverage and

.. math::

   A_{\text{overlap}} = 0.

Planet Fully Inside the Beam
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If the planet disk lies entirely inside the beam,

.. math::

   d \le |R - r|

then the overlap area is simply the area of the planet disk,

.. math::

   A_{\text{overlap}} = \pi r^2.

Partial Overlap
~~~~~~~~~~~~~~~

If the two circles partially overlap,

.. math::

   |R - r| < d < R + r,

the overlap area is computed using the standard closed-form
circle--circle intersection formula (`Weisstein, Eric W. "Circle-Circle Intersection." From MathWorld--A Wolfram Resource. <https://mathworld.wolfram.com/Circle-CircleIntersection.html>`__),

.. math::

   A_{\text{overlap}} =
   R^2 \cos^{-1}\!\left(
      \frac{d^2 + R^2 - r^2}{2 d R}
   \right)
   +
   r^2 \cos^{-1}\!\left(
      \frac{d^2 + r^2 - R^2}{2 d r}
   \right)
   -
   \frac{1}{2}
   \sqrt{
      (-d + R + r)
      ( d + R - r)
      ( d - R + r)
      ( d + R + r)
   }.

Coverage Fraction
~~~~~~~~~~~~~~~~~

The antenna beam coverage fraction due to the planet is obtained by normalizing
the overlap area by the beam area,

.. math::

   C_p = \frac{A_{\text{overlap}}}{\pi R^2}.

Since :math:`R = 1` in normalized beam space, this reduces to

.. math::

   C_p = \frac{A_{\text{overlap}}}{\pi}.

The coverage fraction is clamped to the interval :math:`[0, 1]`.

.. _module-assumptions:

Module Assumptions and Limitations
----------------------------------

The following assumptions and limitations apply to the simpleAntenna module:

- The antennas radiation pattern is modelled as a simple 2D Gaussian beam pattern. This has the following implications\:
  - Sidelobes are neglected, and thus any noise, interference or signal received through sidelobes is not considered.
  - The main-lobe is approximated as a Gaussian function in both elevation and azimuth direction :math:`\rightarrow` The antenna gain is computed as a function of the angle off boresight in both elevation and azimuth direction.

- For the calculation of :math:`T_\mathrm{Sky}` the antenna gain is not used to weight the sky and planet noise temperatures, instead the beam coverage fractions are used (see also :ref:`module-overlap-area-computation`). A uniform gain across the beam is assumed for this step.

- The antenna can be placed either on a spacecraft or on Earth (ground). It is not (yet) possible to place the antenna on other celestial bodies.

- For ground based antennas, the sky noise temperature is always assumed to be :math:`T_\mathrm{Sky} = 200K`. This is because the ground antenna does not have a pointing direction. It is assumed that the ground based antenna is always precisely pointed at the spacecraft antenna.

- Doppler shift between two antennas is not (yet) considered.

- Noise due to lightning and other static electrical discharges is neglected.

- Man-made radio noise is neglected.

- Solar activity variations are neglected.

User Guide
----------

Setting the Antenna Environment
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The antenna environment is determined by which state message is connected. The module must be connected to either a spacecraft state message or a ground state message, but not both.

**For a space-based antenna**, connect the spacecraft state message:

.. code-block:: python

    antenna.scStateInMsg.subscribeTo(scObject.scStateOutMsg)

The spacecraft state message is of type :ref:`SCStatesMsgPayload`.

**For a ground-based antenna**, connect the ground state message:

.. code-block:: python

    antenna.groundStateInMsg.subscribeTo(groundStation.currentGroundStateOutMsg)

The ground state message is of type :ref:`GroundStateMsgPayload`.

Setting Required Antenna Parameters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The following parameters must be set before the simulation is initialized:
(here shown with example values for a typical S-band communication antenna)

.. code-block:: python

    antenna.setAntennaName("CommAntenna")                   # [-]  Antenna name
    antenna.setAntennaFrequency(2.2e9)                      # [Hz] Operating frequency
    antenna.setAntennaBandwidth(5e6)                        # [Hz] Bandwidth
    antenna.setAntennaDirectivity_dB(20.0)                  # [dB] Directivity (must be > 9 dB)
    antenna.setAntennaHpbwRatio(1.0)                        # [-]  Beam shape ratio k
    antenna.setAntennaP_Tx(100.0)                           # [W]  Transmit power
    antenna.setAntennaEquivalentNoiseTemp(50.0)             # [K]  Receiver noise temperature
    antenna.setAntennaRadEfficiency(0.55)                   # [-]  Radiation efficiency

Setting Optional Antenna Parameters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The following parameters are optional:

.. code-block:: python

    antenna.setAntennaP_Rx(10.0)                            # [W]    Receive power (default: 0)
    antenna.setAntennaEnvironmentTemperature(290.0)         # [K]    Ambient temperature (default: calculated based on ISA (ground) or 150K (space))
    antenna.setUseHaslamMap(True)                           # [bool] Enable Haslam sky map (default: False)

.. note::

   Calling :code:`antenna.setUseHaslamMap(True)`` enables the use of the Haslam 408 MHz all-sky survey map for sky noise temperature calculation, and calles the function :code:`configureBrightnessFile(<filepath>)` internally.
   If the user wants to use a custom brightness temperature map file, they must call :code:`antenna.configureBrightnessFile(<custom filePath>)` after calling :code:`antenna.setUseHaslamMap(True)`.

Setting the Antenna Position and Orientation (Space-Based Only)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For space-based antennas, the antenna position and orientation relative to the spacecraft body frame can be set:

.. code-block:: python

    antenna.setAntennaPositionBodyFrame([0.5, 0.0, 0.0])    # [m]    Position offset (default: antenna is at B = [0,0,0])
    antenna.setAntennaOrientationBodyFrame([0.0, 0.0, 0.0]) # [-]    MRP rotation (default: antenna is aligned with body frame)

Setting the Antenna State
^^^^^^^^^^^^^^^^^^^^^^^^^

The antenna state can be set via an input message or directly using a setter. To connect an antenna state input message:

.. code-block:: python

    antenna.antennaSetStateInMsg.subscribeTo(antennaController.antennaStateOutMsg)

The antenna state message is of type :ref:`AntennaStateMsgPayload`.

Alternatively, the state can be set directly using the setter method:

.. code-block:: python

    antenna.setAntennaState(simpleAntenna.AntennaTypes.ANTENNA_RXTX)

Valid states are ``ANTENNA_OFF`` (0), ``ANTENNA_RX`` (1), ``ANTENNA_TX`` (2), and ``ANTENNA_RXTX`` (3). The default state is ``ANTENNA_OFF``.

Adding Celestial Bodies for Sky Noise Calculation (Optional)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For more accurate sky noise temperature calculations, celestial body positions can be provided. To add the sun:

.. code-block:: python

    antenna.sunInMsg.subscribeTo(gravFactory.spiceObject.planetStateOutMsgs[0])

To add planets (e.g., Earth, Moon) for beam coverage calculations, use the ``addPlanetToModel`` method:

.. code-block:: python

    antenna.addPlanetToModel(gravFactory.spiceObject.planetStateOutMsgs[1])  # Earth
    antenna.addPlanetToModel(gravFactory.spiceObject.planetStateOutMsgs[2])  # Moon

The planet state messages are of type :ref:`SpicePlanetStateMsgPayload`.

To account for eclipse conditions affecting solar noise contribution:

.. code-block:: python

    antenna.sunEclipseInMsg.subscribeTo(eclipseObject.eclipseOutMsgs[0])

The eclipse message is of type :ref:`EclipseMsgPayload`.

Antenna Output Message
^^^^^^^^^^^^^^^^^^^^^^

The ``simpleAntenna`` module outputs an antenna state message of type :ref:`AntennaLogMsgPayload`. To subscribe to this output:

.. code-block:: python

    antennaLog = antenna.antennaOutMsg.recorder()
    scSim.AddModelToTask(simTaskName, antennaLog)

After running the simulation, the logged data can be accessed:

.. code-block:: python

    P_eirp = antennaLog.P_eirp_dB   # [dB]   EIRP
    G_T = antennaLog.G_TN           # [dB/K] Figure of merit (G/T)
    P_N = antennaLog.P_N            # [W]    Noise power
    r_AN_N = antennaLog.r_AN_N      # [m]    Antenna position in inertial frame
    sigma_AN = antennaLog.sigma_AN  # [-]    Antenna orientation (MRP)
