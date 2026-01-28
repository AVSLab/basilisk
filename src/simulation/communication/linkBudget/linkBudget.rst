.. Warning::

   **[BETA]** :ref:`linkBudget`  is a beta module in an initial public release. This module might be
   subject to changes in future releases.

Executive Summary
-----------------
This document describes how the linkBudget module operates within the Basilisk astrodynamics simulation framework. The purpose of this module is to provide a radio link budget calculation between two antennas, enabling the simulation of spacecraft-to-spacecraft or spacecraft-to-ground communication links.

The module computes the end-to-end link budget accounting for free space path loss (FSPL), atmospheric attenuation (based on `ITU-R P.676 <https://www.itu.int/rec/R-REC-P.676>`__), antenna pointing losses, and frequency offset losses. The primary output is the Carrier-to-Noise Ratio (CNR) for each receiving antenna, which is essential for evaluating communication system performance.

This module requires two connected antenna output messages (e.g. from :ref:`simpleAntenna` modules). The link budget calculation automatically determines the link configuration (space-to-space or space-to-ground) based on the antenna environment types.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.
The module msg connection is set by the user from python.
The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 20 25 35 20
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
      - Note
    * - antennaInPayload_1
      - :ref:`AntennaLogMsgPayload`
      - Output message from antenna 1
      - Required
    * - antennaInPayload_2
      - :ref:`AntennaLogMsgPayload`
      - Output message from antenna 2
      - Required
    * - linkBudgetOutPayload
      - :ref:`LinkBudgetMsgPayload`
      - Link budget calculation results
      - Output

.. _detailed-module-description:

Detailed Module Description
---------------------------
The linkBudget module calculates the radio link budget between two antennas. It receives antenna state information from two antenna modules (e.g. :ref:`simpleAntenna`) and computes the various losses and the resulting Carrier-to-Noise Ratio (CNR) for each receiving antenna.

The module supports the following communication configurations:

- **Space <--> Space**: Direct line-of-sight communication between two spacecraft
- **Space <--> Ground**: Communication between a spacecraft and a ground station on Earth

**Ground <--> Ground** communication and communication with a Ground station not on Earth are not supported (see :ref:`module-assumptions-link`).

Link Budget Calculation Overview
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The link budget equation implemented by this module is\:

.. math::

   P_\mathrm{Rx}\,\mathrm{[dBW]} = P_\mathrm{EIRP} + G_\mathrm{Rx} - L_\mathrm{FSPL} - L_\mathrm{atm} - L_\mathrm{point} - L_\mathrm{freq}

where\:

- :math:`P_\mathrm{Rx}` is the received power at the receiver antenna in :math:`[\mathrm{dBW}]`
- :math:`P_\mathrm{EIRP}` is the Equivalent Isotropically Radiated Power (EIRP) of the transmitter antenna in :math:`[\mathrm{dBW}]`
- :math:`G_\mathrm{Rx}` is the receiver antenna gain in :math:`[\mathrm{dBi}]`
- :math:`L_\mathrm{FSPL}` is the Free Space Path Loss (FSPL) in :math:`[\mathrm{dB}]`
- :math:`L_\mathrm{atm}` is the atmospheric attenuation loss in :math:`[\mathrm{dB}]`
- :math:`L_\mathrm{point}` is the pointing error loss in :math:`[\mathrm{dB}]`
- :math:`L_\mathrm{freq}` is the frequency offset loss in :math:`[\mathrm{dB}]`

The following sections describe each loss term in detail.

.. _fspl-calculation:

Free Space Path Loss (FSPL)
^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. _Figure 1:
.. figure:: /../../src/simulation/communication/linkBudget/_Documentation/Images/LinkBudgetFspl.svg
   :width: 40%
   :align: center
   :alt: Schematic of Free Space Path Loss calculation

   Figure 1: Illustration of Free Space Path Loss (FSPL) between two antennas

The Free Space Path Loss (FSPL) accounts for the signal power reduction due to the spreading of electromagnetic waves over distance. This is often the dominant loss term in space communication links.
The FSPL is illustrated in Figure 1, showing two antennas separated by distance :math:`d`.

.. math::

   L_\mathrm{FSPL}\,\mathrm{[dB]} = 20 \cdot \log_{10}\left(\frac{4 \pi d}{\lambda}\right)

where\:

- :math:`d` is the distance between the two antennas :math:`[\mathrm{m}]`
- :math:`\lambda = c / f` is the wavelength :math:`[\mathrm{m}]`, with :math:`c` :math:`[\mathrm{m/s}]` being the speed of light and :math:`f` :math:`[\mathrm{Hz}]` the center frequency of the overlapping bandwidth.

The distance :math:`d` is computed from the antenna position vectors :math:`\mathbf{r}_\mathrm{A1,N}^\mathrm{N}` and :math:`\mathbf{r}_\mathrm{A2,N}^\mathrm{N}` provided in the input messages\:

.. math::

   d = \|\mathbf{r}_\mathrm{A1,N}^\mathrm{N} - \mathbf{r}_\mathrm{A2,N}^\mathrm{N}\|

.. _atmospheric-attenuation:

Atmospheric Attenuation
^^^^^^^^^^^^^^^^^^^^^^^

.. _Figure 2:
.. figure:: /../../src/simulation/communication/linkBudget/_Documentation/Images/LinkBudgetAtm.svg
   :width: 40%
   :align: center
   :alt: Schematic of atmospheric attenuation along slant path

   Figure 2: Illustration of atmospheric attenuation calculation along the slant path from ground to spacecraft

For **space-to-ground** links, the module computes atmospheric attenuation due to oxygen, water vapor and nitrogen absorption following the `ITU-R P.676-13 <https://www.itu.int/rec/R-REC-P.676>`__ recommendation. This calculation is only performed when atmospheric attenuation is enabled ``atmosAtt == True`` and one antenna is located on the ground.

The atmospheric attenuation model divides the atmosphere into discrete layers (increasing thickness with increasing altitude) from the ground antenna altitude up to 100 km (Karman line). The slant path depending on angle :math:`\alpha` (see Figure 2) through each layer is computed based on the antenna positions and the altitude of each layer.

The atmospheric conditions (temperature, pressure, water vapor density) at each altitude layer are obtained from the `ITU-R P.835-7 <https://www.itu.int/rec/R-REC-P.835>`__ reference standard atmosphere model via the ``ItuAtmosphere`` utility class.

.. note::
   Atmospheric attenuation lookup tables are precomputed at initialization, using the ground antenna position and frequency and re-used during simulation. Changes in frequency during simulation will not update the atmospheric attenuation profile.

For more information the reader is referred to the following ITU-R recommendations:

- `ITU-R P.676-13 <https://www.itu.int/rec/R-REC-P.676>`__: Attenuation by atmospheric gases and related effects
- `ITU-R P.835-7 <https://www.itu.int/rec/R-REC-P.835>`__: Reference standard atmospheres
- `ITU-R P.453-14 <https://www.itu.int/rec/R-REC-P.453>`__: The radio refractive index: its formula and refractivity data

.. _pointing-loss:

Pointing Loss
^^^^^^^^^^^^^

.. _Figure 3:
.. figure:: /../../src/simulation/communication/linkBudget/_Documentation/Images/LinkBudgetPoint.svg
   :width: 40%
   :align: center
   :alt: Schematic of antenna pointing loss

   Figure 3: Illustration of pointing loss due to antenna misalignment (pointing error angle in elevation, :math:`\| \theta_\mathrm{el} \| > 0^\circ`)

The pointing error loss accounts for signal degradation when the antennas are not perfectly aligned. This loss is computed assuming a Gaussian antenna radiation pattern (consistent with the :ref:`simpleAntenna` module).

For each antenna, the pointing error is decomposed into azimuth and elevation components relative to the antenna boresight direction\:

According to the :ref:`simpleAntenna` gaussian-beam pattern, the pointing loss is computed as\:

.. math::

   L_\mathrm{point}\,\mathrm{[dB]} = 10 \cdot \log_{10}(e) \cdot 4 \cdot \ln(2) \cdot \left[ \left(\frac{\theta_\mathrm{az}}{\theta_\mathrm{HPBW,az}}\right)^2 + \left(\frac{\theta_\mathrm{el}}{\theta_\mathrm{HPBW,el}}\right)^2 \right]

where\:

- :math:`e` is Euler's number :math:`[-]`
- :math:`\theta_\mathrm{az}` is the azimuth pointing error :math:`[\mathrm{rad}]`
- :math:`\theta_\mathrm{el}` is the elevation pointing error :math:`[\mathrm{rad}]`
- :math:`\theta_\mathrm{HPBW,az}` is the Half-Power BeamWidth in azimuth :math:`[\mathrm{rad}]`
- :math:`\theta_\mathrm{HPBW,el}` is the Half-Power BeamWidth in elevation :math:`[\mathrm{rad}]`

For **space-to-space** links, the total pointing loss is the sum of pointing losses from both antennas. For **space-to-ground** links, the ground antenna is assumed to be perfectly pointed at the spacecraft, so only the spacecraft antenna pointing loss is computed.

.. note::
   If a **custom antenna** module is implemented (non gaussian radiation pattern), the pointing loss calculation in the linkBudget module must be updated accordingly!

.. _frequency-offset-loss:

Frequency Offset Loss
^^^^^^^^^^^^^^^^^^^^^

.. _Figure 4:
.. figure:: /../../src/simulation/communication/linkBudget/_Documentation/Images/LinkBudgetBand.svg
   :width: 30%
   :align: center
   :alt: Schematic of frequency offset and bandwidth overlap

   Figure 4: Illustration of frequency offset loss due to partial bandwidth overlap

The frequency offset loss accounts for the reduction in effective bandwidth when the two antennas operate at slightly different center frequencies or have different bandwidths.

The overlapping bandwidth is computed as\:

.. math::

   B_\mathrm{overlap} = f_\mathrm{high} - f_\mathrm{low}

where\:

.. math::

   f_\mathrm{low} = \max\left(f_\mathrm{Tx} - \frac{B_\mathrm{Tx}}{2}, f_\mathrm{Rx} - \frac{B_\mathrm{Rx}}{2}\right)

   f_\mathrm{high} = \min\left(f_\mathrm{Tx} + \frac{B_\mathrm{Tx}}{2}, f_\mathrm{Rx} + \frac{B_\mathrm{Rx}}{2}\right)

The frequency offset loss is then\:

.. math::

   L_\mathrm{freq}\,\mathrm{[dB]} = 10 \cdot \log_{10}\left(\frac{B_\mathrm{overlap}}{B_\mathrm{min}}\right)

where :math:`B_\mathrm{min} = \min(B_\mathrm{Tx}, B_\mathrm{Rx})` is the smaller of the two antenna bandwidths.

- :math:`f_\mathrm{high}` is the upper frequency of the overlapping bandwidth :math:`[\mathrm{Hz}]` (see Figure 4)
- :math:`f_\mathrm{low}` is the lower frequency of the overlapping bandwidth :math:`[\mathrm{Hz}]` (see Figure 4)
- :math:`f_\mathrm{Tx}` is the transmitter antenna center frequency :math:`[\mathrm{Hz}]`
- :math:`f_\mathrm{Rx}` is the receiver antenna center frequency :math:`[\mathrm{Hz}]`
- :math:`B_\mathrm{Tx}` is the transmitter antenna bandwidth :math:`[\mathrm{Hz}]`
- :math:`B_\mathrm{Rx}` is the receiver antenna bandwidth :math:`[\mathrm{Hz}]`

If there is no overlapping bandwidth (:math:`B_\mathrm{overlap} \leq 0`), the radio link is considered non-functional which is reflected in the link budget calculation.

.. _cnr-calculation:

Carrier-to-Noise Ratio (CNR)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The Carrier-to-Noise Ratio is the primary output of this module for evaluating link performance. It is computed for each antenna that is in receive mode (``ANTENNA_RX`` or ``ANTENNA_RXTX``)\:

.. math::

   \mathrm{CNR}\,\mathrm{[-]} = 10^{(P_\mathrm{Rx} - P_\mathrm{N}) / 10}

where\:

- :math:`P_\mathrm{Rx}` is the received signal power [dBW]
- :math:`P_\mathrm{N}` is the noise power from the antenna module [dBW]

If an antenna is not in receive mode, its CNR is set to :math:`0.0`.

Module Configuration
^^^^^^^^^^^^^^^^^^^^

The following optional settings can be configured\:

.. list-table:: Module Configuration Options
   :widths: 20 15 15 50
   :header-rows: 1

   * - Parameter
     - Default
     - Type
     - Description
   * - atmosAtt
     - ``false``
     - bool
     - Enable/disable atmospheric attenuation calculation (only applicable for space-ground links)
   * - pointingLoss
     - ``true``
     - bool
     - Enable/disable pointing loss calculation
   * - freqLoss
     - ``true``
     - bool
     - Enable/disable frequency offset loss calculation

   :math:`L_\mathrm{FSPL}` is always calculated and cannot be disabled.

.. _module-assumptions-link:

Module Assumptions and Limitations
----------------------------------

The following assumptions and limitations apply to the linkBudget module\:

- **Ground-to-ground communication is not supported.** Both antennas cannot be in a ground environment simultaneously.

- **Atmospheric attenuation is based on a clear-sky model.** The following effects are not included:

  - Rain attenuation (`ITU-R P.838 <https://www.itu.int/rec/R-REC-P.838>`__)
  - Cloud and fog attenuation (`ITU-R P.840 <https://www.itu.int/rec/R-REC-P.840>`__)
  - Ionospheric scintillation effects (`ITU-R P.618 <https://www.itu.int/rec/R-REC-P.618>`__)
  - Other ionospheric effects (`ITU-R P.531 <https://www.itu.int/rec/R-REC-P.531>`__)
  - Atmospheric signal path bending (`ITU-R P.453 <https://www.itu.int/rec/R-REC-P.453>`__)

- **The International Telecommunication Union's (ITU) Reference Standard Atmosphere (RSA) is used for atmospheric attenuation.** Local atmospheric variations and seasonal changes as e.g. shown in `ITU-R P.835 <https://www.itu.int/rec/R-REC-P.835>`__ are not modeled.

- **The atmospheric attenuation model is valid for frequencies between 1 GHz and 1000 GHz.** This range is defined by the ITU-R P.676-13 recommendation. Frequencies outside this range may produce inaccurate atmospheric attenuation estimates.

- **Atmospheric attenuation calculations require a minimum elevation angle of: :math:`5^\circ`.** At lower elevation angles (grazing angles), the slant path through the atmosphere becomes very long, leading to numerical instabilities and potentially unrealistic attenuation values. The module clamps the elevation angle to a minimum of 5° for atmospheric calculations.

- **Ground antennas can only be placed on Earth.** The model does not currently support ground stations on other celestial bodies.

- **Ground antennas are assumed to be perfectly pointed.** For space-to-ground links, only the spacecraft antenna contributes to pointing loss.

- **Doppler shift due to spacecraft motion is currently not considered.** The frequency offset between antennas due to relative motion is not currently computed.

- **Impedance mismatch losses are not included.**

- **Polarization is not modeled.**

- **Communication between close antennas (< 10 km) may yield inaccurate results.** The Gaussian-beam assumption in :ref:`simpleAntenna` may not get accurate results at close range.

- **Multipath effects are not considered.**

Technical Notes
----------------

Frequency Range
^^^^^^^^^^^^^^^

The atmospheric attenuation model implements the line-by-line calculation method from ITU-R P.676-13, which is valid for frequencies from **1 GHz to 1000 GHz**. The model includes:

- 44 oxygen absorption lines (Table 1 of ITU-R P.676-13)
- 35 water vapor absorption lines (Table 2 of ITU-R P.676-13)
- Pressure-induced nitrogen absorption continuum
- Non-resonant Debye spectrum for oxygen below 10 GHz

Elevation Angle Constraints
^^^^^^^^^^^^^^^^^^^^^^^^^^^

The atmospheric slant path integration uses the relationship:

.. math::

   A_\mathrm{gas} = \int_{h_1}^{h_2} \frac{\gamma(h)}{\sin \phi(h)} \, dh

where :math:`\phi(h)` is the elevation angle at height :math:`h`. As :math:`\phi \to 0^\circ`, the denominator approaches zero, causing numerical issues.

To ensure numerical stability, the module enforces a **minimum elevation angle of :math:`5^\circ`** for atmospheric attenuation calculations. This corresponds to a maximum air mass factor of approximately 11.5.

.. list-table:: Elevation Angle and Air Mass Factor
   :widths: 25 25 50
   :header-rows: 1

   * - Elevation Angle
     - Air Mass Factor
     - Notes
   * - :math:`90^\circ` (zenith)
     - 1.0
     - Minimum path length
   * - :math:`30^\circ`
     - 2.0
     -
   * - :math:`10^\circ`
     - 5.8
     -
   * - :math:`5^\circ`
     - 11.5
     - Module minimum
   * - :math:`0^\circ` (horizon)
     - inf
     - Not supported

User Guide
----------

Setting Up the Link Budget Module
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To use the linkBudget module, you must first create two antenna modules (using :ref:`simpleAntenna`) and connect their output messages to the link budget module.

.. code-block:: python

    from Basilisk.simulation import linkBudget, simpleAntenna
    from Basilisk.utilities import SimulationBaseClass

    # Create simulation
    scSim = SimulationBaseClass.SimBaseClass()

    # Create process and task
    dynProcess = scSim.CreateNewProcess("DynamicsProcess")
    dynTaskName = "dynTask"
    dynProcess.addTask(scSim.CreateNewTask(dynTaskName, int(1E9)))  # 1 second update rate

    # Create antenna modules
    antenna1 = simpleAntenna.SimpleAntenna()
    antenna1.ModelTag = "antenna1"
    # ... configure antenna1 parameters ...

    antenna2 = simpleAntenna.SimpleAntenna()
    antenna2.ModelTag = "antenna2"
    # ... configure antenna2 parameters ...

    # Create link budget module
    linkBudgetModule = linkBudget.LinkBudget()
    linkBudgetModule.ModelTag = "linkBudget"

    # Connect antenna output messages to link budget inputs
    linkBudgetModule.antennaInPayload_1.subscribeTo(antenna1.antennaOutStateMsg)
    linkBudgetModule.antennaInPayload_2.subscribeTo(antenna2.antennaOutStateMsg)

    # Add modules to task
    scSim.AddModelToTask(dynTaskName, antenna1)
    scSim.AddModelToTask(dynTaskName, antenna2)
    scSim.AddModelToTask(dynTaskName, linkBudgetModule)

Enabling Atmospheric Attenuation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Atmospheric attenuation is disabled by default. To enable it for space-to-ground links, set the ``atmosAtt`` flag to True before running the simulation:

.. code-block:: python

    # Enable atmospheric attenuation for space-ground links
    linkBudgetModule.atmosAtt = True

    # Optionally disable other loss calculations
    linkBudgetModule.pointingLoss = False  # Disable pointing loss
    linkBudgetModule.freqLoss = False      # Disable frequency offset loss

.. note::

   Atmospheric attenuation is automatically disabled for space-to-space links regardless of the configuration setting.

.. warning::

   The atmospheric attenuation lookup table is precomputed during module initialization using the ground antenna's initial position and frequency. Changes to the antenna frequency during simulation will **not** update the atmospheric attenuation profile. If frequency changes are required, the simulation must be re-initialized.

Accessing Link Budget Results
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The link budget module outputs a message of type :ref:`LinkBudgetMsgPayload`. To subscribe to and record this output\:

.. code-block:: python

    # Set up message recorder
    linkBudgetLog = linkBudgetModule.linkBudgetOutPayload.recorder()
    scSim.AddModelToTask(dynTaskName, linkBudgetLog)

    # Run simulation
    scSim.InitializeSimulation()
    scSim.ExecuteSimulation()

    # Access logged data
    CNR1 = linkBudgetLog.CNR1          # [-]  Carrier-to-Noise Ratio for antenna 1
    CNR2 = linkBudgetLog.CNR2          # [-]  Carrier-to-Noise Ratio for antenna 2
    distance = linkBudgetLog.distance  # [m]  Distance between antennas
    bandwidth = linkBudgetLog.bandwidth # [Hz] Overlapping bandwidth
    frequency = linkBudgetLog.frequency # [Hz] Center frequency

Accessing Individual Loss Terms
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The individual loss terms can be accessed through getter methods for debugging or analysis purposes\:

.. code-block:: python

    L_FSPL = linkBudgetModule.getL_FSPL()   # [dB] Free space path loss
    L_atm = linkBudgetModule.getL_atm()     # [dB] Atmospheric attenuation
    L_point = linkBudgetModule.getL_point() # [dB] Pointing loss
    L_freq = linkBudgetModule.getL_freq()   # [dB] Frequency offset loss

References
----------

The atmospheric attenuation model is based on the following ITU-R recommendations\:

- `ITU-R P.676-13 <https://www.itu.int/rec/R-REC-P.676>`__: Attenuation by atmospheric gases and related effects
- `ITU-R P.835-7 <https://www.itu.int/rec/R-REC-P.835>`__: Reference standard atmospheres
- `ITU-R P.453-14 <https://www.itu.int/rec/R-REC-P.453>`__: The radio refractive index: its formula and refractivity data
