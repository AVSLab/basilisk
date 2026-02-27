Executive Summary
-----------------
The ``spacecraftChargingEquilibrium`` module computes spacecraft equilibrium (floating) electric potentials in a plasma environment using two sequential **1-D root solves** (by enforcing a zero net current condition on each spacecraft surface). The model includes ambient plasma electron/ion collection, photoelectron emission, and secondary electron emission (SEE), electron backscattering, and electron beam coupling (from one spacecraft to another).

The charging and electron-beam coupling equations used by this module follow Hammerl and Schaub’s coupled spacecraft charging model [Hammerl2024]_ and the references cited therein as well as the Dr. Lai's book: "Fundamentals of Spacecraft Charging". Additional background references on plasma interactions and modeling include [Hippler2001]_, [DavisMandell2016]_, [DraineSalpeter1979]_, and [RomeroCalvo2022]_.


A Python unit test script is provided to verify expected charging trends for a coupled two-spacecraft case (see User Guide).

Module Assumptions and Limitations
----------------------------------
- **Conducting spacecraft and single potential**: the theory used assumes each spacecraft is fully conducting and can be described by a single surface potential :math:`\phi` (no differential charging).
- **Exactly two spacecraft are required**: the coupled electron-beam charging logic is implemented for exactly two spacecraft and assumes:

  - spacecraft index ``0`` is the **servicer**
  - spacecraft index ``1`` is the **target**

- **Plasma environment input required**: the plasma flux/energy grid message must be connected. If not connected, the module logs an error in ``Reset()``.
- **Surface areas**: the module uses a configurable default exposed area and a configurable sunlit-area fallback. Sunlit area can be provided per spacecraft through input messages or per-spacecraft defaults via setters.
- **Numerical solution**: equilibrium potentials are found using a bisection root solver. The bracket range must contain the root for each current balance equation.


Message Connection Descriptions
-------------------------------
The following table lists all module input and output messages. The module msg variable name is set by the user
from Python. The msg type contains a link to the message structure definition, while the description explains how
the message is used.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - plasmaFluxInMsg
      - :cpp:struct:`PlasmaFluxMsgPayload`
      - plasma environment input message containing the energy grid and particle flux distributions used to compute ambient electron/ion collection currents and yield-weighted emission currents
    * - scStateInMsgs
      - :cpp:struct:`SCStatesMsgPayload`
      - vector of spacecraft state input messages; entries are appended via ``addSpacecraft()``
    * - eBeamInMsgs
      - :cpp:struct:`ElectronBeamMsgPayload`
      - vector of electron beam parameter input messages (optional). If a spacecraft’s beam message is linked, its gun parameters are read from the message and used in beam-coupled charging terms
    * - voltOutMsgs
      - :cpp:struct:`VoltMsgPayload`
      - output vector of equilibrium spacecraft potentials (voltage), one per spacecraft
    * - currentsOutMsgs
      - :cpp:struct:`ScChargingCurrentsMsgPayload`
      - output vector of per-spacecraft current-component breakdowns used in the equilibrium evaluation (ambient, beam, and total terms)
    * - scSunlitAreaInMsgs
      - :cpp:struct:`ScSunlitFacetAreaMsgPayload`
      - vector of optional sunlit facet area input messages, one per spacecraft. If linked and written, this overrides the configured per-spacecraft default or fallback sunlit area used in charging calculations.


Detailed Module Description
---------------------------

Overview: zero net current equilibrium
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
The module computes equilibrium potential(s) by requiring the **total current** to each spacecraft to be zero:

.. math::

    I_{\mathrm{tot}}(\phi) = \sum_k I_k(\phi) = 0

For the servicer/target problem with electron beam emission and impact, the total currents
for the servicer and target are broken down into the terms in the following section and the equilibrium is solved by root-finding these current-balance
equations numerically.

A note on implementation style
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Hammerl (2024) presents closed-form expressions for Maxwellian environments (e.g., ambient collection currents) and also defines yield-weighted current forms that can be evaluated from an energy distribution. The Basilisk implementation uses the same **energy-shifted flux structure** but evaluates currents with:

- linear interpolation of tabulated flux/yield curves
- trapezoidal integration (``trapz()``) over a specified energy interval

This approach is intended to *mimic the behavior of the analytical model* while allowing non-Maxwellian or numerically provided flux distributions.


Ambient collection and emission currents
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Plasma electron collection current
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The implementation evaluates the plasma electron collection current directly from the **tabulated electron flux distribution** :math:`F_e(E)` using an energy-shifted integral. This is consistent with the flux-transform structure used in yield-weighted current expressions (Eq. (5) in [Hammerl2024]_), but here it is applied to the collection current itself.

The current is computed as:

.. math::

    I_e(\phi) = -q\,A \int_{E_{\min}}^{E_{\max}} \left(\frac{E}{E-\phi}\right)\,F_e(E-\phi)\,dE

where :math:`A` is the area exposed to plasma, :math:`q` is the elementary charge, and :math:`F_e(E)` is the electron flux distribution provided by ``plasmaFluxInMsg``.

Numerical details (as implemented)
""""""""""""""""""""""""""""""""""
- The integrand used in code is::

    (E/(E - phi)) * getFlux(E - phi, "electron")

- The integration is performed by trapezoidal quadrature (``trapz``).
- The integration bounds are chosen to keep the shifted argument :math:`(E-\phi)` inside the tabulated energy range and to avoid evaluating too close to the singular point:

  - If :math:`\phi < 0`, bounds are approximately:

    .. math::
        E_{\min} \approx 0.1,\quad E_{\max} \approx E_{\mathrm{grid,max}}

  - If :math:`\phi \ge 0`, bounds are shifted upward:

    .. math::
        E_{\min} \approx 0.1 + |\phi|,\quad E_{\max} \approx E_{\mathrm{grid,max}} + |\phi|

This numerical choice is designed to mimic the potential-shift behavior of the analytical model while remaining stable for arbitrary tabulated flux distributions.


Plasma ion collection current
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The ion collection current is computed in the same numerical style using the tabulated ion flux distribution :math:`F_i(E)`:

.. math::

    I_i(\phi) = +q\,A \int_{E_{\min}}^{E_{\max}} \left(\frac{E}{E+\phi}\right)\,F_i(E+\phi)\,dE

Numerical details (as implemented)
""""""""""""""""""""""""""""""""""
- The integrand used in code is::

    (E/(E + phi)) * getFlux(E + phi, "ion")

- The bounds are chosen so that :math:`(E+\phi)` remains within the tabulated range:

  - If :math:`\phi > 0`:

    .. math::
        E_{\min} \approx 0.1,\quad E_{\max} \approx E_{\mathrm{grid,max}}

  - If :math:`\phi \le 0`:

    .. math::
        E_{\min} \approx 0.1 + |\phi|,\quad E_{\max} \approx E_{\mathrm{grid,max}} + |\phi|

These bounds mirror the logic in the implementation and help avoid evaluating near :math:`E+\phi=0`.


Secondary electron emission due to ambient electrons
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
SEE current due to ambient electrons is computed by weighting the electron flux by the electron SEE yield curve and applying the same energy-shift/Jacobian factor. This corresponds to the yield-weighting structure described in [Hammerl2024]_ (Eq. (5) and the related discussion), evaluated numerically:

.. math::

    I_{\mathrm{SEE,e}}(\phi) = q\,A \int Y_e(E)\left(\frac{E}{E-\phi}\right)\,F_e(E-\phi)\,dE

For :math:`\phi>0`, the emitted secondaries can be recollected, and the implementation applies an exponential suppression factor consistent with the form used in Hammerl (Eq. (3) in [Hammerl2024]_):

.. math::

    I_{\mathrm{SEE,e}}(\phi>0) = I_{\mathrm{SEE,e}}(\phi)\exp\!\left(-\frac{\phi}{T_{\mathrm{SEE}}}\right)

where ``secondaryElectronTemperature`` is the model parameter for secondary electron temperature.


Secondary electron emission due to ambient ions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The ion-driven SEE current is evaluated using the tabulated ion yield curve:

.. math::

    I_{\mathrm{SEE,i}}(\phi) = q\,A \int Y_i(E)\left(\frac{E}{E+\phi}\right)\,F_i(E+\phi)\,dE

and uses the same :math:`\exp(-\phi/T_{\mathrm{SEE}})` suppression for :math:`\phi>0`.


Electron backscattering current due to ambient electrons
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Backscattering due to ambient electrons is computed using the backscatter yield curve with the same shifted-flux structure:

.. math::

    I_{\mathrm{BS}}(\phi) = q\,A \int Y_{\mathrm{BS}}(E)\left(\frac{E}{E-\phi}\right)\,F_e(E-\phi)\,dE

and applies :math:`\exp(-\phi/T_{\mathrm{BS}})` suppression for :math:`\phi>0`, where
``backscatterElectronTemperature`` is the corresponding model parameter.


Photoelectric current
^^^^^^^^^^^^^^^^^^^^^
The photoelectric current is implemented in a standard piecewise form (Eq. (10) in [Hammerl2024]_), with the underlying functional form commonly referenced to spacecraft charging texts such as [Lai2011]_:

.. math::

    I_{\mathrm{ph}}(\phi) =
    \begin{cases}
      j_{\mathrm{ph},0}\,A_{\mathrm{sunlit}}, & \phi \le 0 \\
      j_{\mathrm{ph},0}\,A_{\mathrm{sunlit}} \exp\!\left(-\dfrac{\phi}{T_{\mathrm{ph}}}\right), & \phi > 0
    \end{cases}

In code, ``photoelectronFlux`` corresponds to :math:`j_{\mathrm{ph},0}` and ``photoelectronTemperature`` corresponds to :math:`T_{\mathrm{ph}}`.


Electron beam coupling currents
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Effective energy for beam-induced yields
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Beam electrons arriving at the target use the effective (landing) energy defined by the potential difference (Eq. (8) in [Hammerl2024]_):

.. math::

    E_{\mathrm{eff}} = E_{\mathrm{EB}} - \phi_S + \phi_T

This value is used when evaluating yield curves for beam-induced SEE and backscattering.


Electron beam current on the target
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The target beam current follows Hammerl’s reachability condition and exponential turn-on (Eq. (11) in [Hammerl2024]_):

.. math::

    I_{\mathrm{EB,T}}(\phi_T,\phi_S) =
    \begin{cases}
      -\alpha I_{\mathrm{EB}}\left(1-\exp\!\left(-\dfrac{E_{\mathrm{EB}}-\phi_S+\phi_T}{T_{\mathrm{EB}}}\right)\right), & E_{\mathrm{EB}} > \phi_S-\phi_T \\
      0, & E_{\mathrm{EB}} \le \phi_S-\phi_T
    \end{cases}

where :math:`\alpha` is modeled as a geometry factor representing the fraction of Ebeam current reaching the target (provided as ``alphaEB`` in the message payload, constrained to :math:`[0,1]` at runtime), and :math:`T_{\mathrm{EB}}` is represented by the model parameter ``beamElectronTemperature``.


Electron beam current on the servicer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The servicer beam current uses the corresponding form (Eq. (13) in [Hammerl2024]_):

.. math::

    I_{\mathrm{EB,S}}(\phi_T,\phi_S) =
    \begin{cases}
      I_{\mathrm{EB}}\left(1-\exp\!\left(-\dfrac{E_{\mathrm{EB}}-\phi_S+\phi_T}{T_{\mathrm{EB}}}\right)\right), & E_{\mathrm{EB}} > \phi_S-\phi_T \\
      0, & E_{\mathrm{EB}} \le \phi_S-\phi_T
    \end{cases}


Beam-induced SEE and backscattering on the target
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Beam-induced SEE and backscattering are implemented by evaluating the yield at :math:`E_{\mathrm{eff}}` and scaling by the magnitude of incoming target beam current:

.. math::

    I_{\mathrm{SEE,eb}} \approx Y_e(E_{\mathrm{eff}})\,\lvert I_{\mathrm{EB,T}}\rvert,\qquad
    I_{\mathrm{BS,eb}} \approx Y_{\mathrm{BS}}(E_{\mathrm{eff}})\,\lvert I_{\mathrm{EB,T}}\rvert

Implementation note:
  Hammerl (2024) includes a recollection factor for emitted secondaries for :math:`\phi_T \ge 0` (see Eq. (12) in [Hammerl2024]_). The current implementation uses yield-scaling directly; adding an explicit :math:`\exp(-\phi_T/T_{\mathrm{SEE}})` factor is straightforward future work if tighter agreement is required.


Servicer/target equilibrium solve workflow
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
The implementation uses a servicer-first sequential solve that follows the modeling workflow used in this module:

.. math::

    I_{\mathrm{tot,S}}(\phi_S;\phi_{T,\mathrm{ref}})=0,\qquad
    I_{\mathrm{tot,T}}(\phi_T;\phi_S)=0

The implementation solves these sequentially:

#. Solve the servicer equilibrium potential :math:`\phi_S` by finding the root of the servicer total-current function, using :math:`\phi_{T,\mathrm{ref}} = 0\,\mathrm{V}` in the servicer beam term.
#. Fix :math:`\phi_S` and solve the target equilibrium potential :math:`\phi_T` by finding the root of the target total-current function.
#. Publish the resulting potentials to ``voltOutMsgs``.

This sequential approach keeps the servicer and target solves internally consistent with the implemented current expressions, while the specific numerical details (integration bounds, discretization, and bracketing choices) are selected to be stable for tabulated flux inputs.


Numerical utilities
~~~~~~~~~~~~~~~~~~~

Linear interpolation (``interp``)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Fluxes and yields are evaluated at arbitrary energies using linear interpolation over the tabulated energy grid. Negative values resulting from extrapolation are clamped to zero in ``getFlux()`` and ``getYield()`` to avoid nonphysical contributions.

Trapezoidal integration (``trapz``)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Current integrals are evaluated using trapezoidal quadrature:

.. math::

    \int_a^b f(x)\,dx \approx h\left(\frac{1}{2}f(a)+\sum_{i=1}^{N-1}f(a+i h)+\frac{1}{2}f(b)\right),\quad
    h = \frac{b-a}{N}

The implementation includes checks for finite bounds and non-degenerate intervals.


User Guide
----------

Creating and adding the module
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
.. code-block:: python

    from Basilisk.simulation import spacecraftChargingEquilibrium
    from Basilisk.utilities import SimulationBaseClass, macros

    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("proc")
    task = sim.CreateNewTask("task", macros.sec2nano(1.0))
    proc.addTask(task)

    chg = spacecraftChargingEquilibrium.SpacecraftChargingEquilibrium()
    chg.ModelTag = "spacecraftChargingEquilibrium"
    sim.AddModelToTask(task.Name, chg)

Connecting the plasma environment
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
.. code-block:: python

    chg.plasmaFluxInMsg.subscribeTo(plasmaMsg)

Adding spacecraft (servicer first, target second)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
The coupled-beam logic assumes:

- ``scStateInMsgs[0]`` is the servicer
- ``scStateInMsgs[1]`` is the target

.. code-block:: python

    chg.addSpacecraft(scServicer.scStateOutMsg)   # index 0
    chg.addSpacecraft(scTarget.scStateOutMsg)     # index 1

Connecting electron beam parameters
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Create and publish an :cpp:struct:`ElectronBeamMsgPayload` and connect it to the desired spacecraft index. Most coupled runs connect the beam parameters on the servicer (index 0):

.. code-block:: python

    from Basilisk.architecture import messaging

    beamPayload = messaging.ElectronBeamMsgPayload()
    beamPayload.energyEB  = 20000.0   # [eV]
    beamPayload.currentEB = 1.0e-6    # [A]
    beamPayload.alphaEB   = 1.0       # fraction of Ebeam current reaching the target

    beamMsg = messaging.ElectronBeamMsg().write(beamPayload)
    chg.eBeamInMsgs[0].subscribeTo(beamMsg)

If the beam message is not linked, the spacecraft is treated as not emitting an electron beam.

Configuring optional model parameters
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
The module provides setters for configurable defaults and numerical settings:

Default construction values
^^^^^^^^^^^^^^^^^^^^^^^^^^^
The table below summarizes the main defaults used at construction time.

.. list-table:: Module Default Values
    :widths: 35 20 45
    :header-rows: 1

    * - Quantity
      - Default
      - How to change
    * - ``enableDebugPrints``
      - ``False``
      - ``setEnableDebugPrints(bool)``
    * - Surface area used in current equations
      - ``50.264 m^2`` (``4*pi*r^2`` for ``r = 2 m``)
      - ``setSurfaceAreaDefault(double)``
    * - Sunlit-area fallback
      - ``0.0 m^2``
      - ``setSunlitAreaFallback(double)``
    * - Per-spacecraft sunlit-area defaults
      - unset (NaN sentinel)
      - ``setSunlitAreaDefault(index, area)`` / ``clearSunlitAreaDefault(index)``
    * - Root solve lower/upper bracket
      - ``[-4.0e5, 4.0e5] V``
      - ``setRootSolveBounds(lower, upper)``
    * - Servicer solve target reference potential
      - ``0.0 V``
      - fixed internal default
    * - Bisection tolerance (servicer / target)
      - ``1e-6 V`` / ``1e-8 V``
      - fixed internal defaults
    * - Trapezoidal integration bins
      - ``1000``
      - ``setTrapzBins(int)``
    * - Minimum integration energy
      - ``0.1 eV``
      - fixed internal default
    * - Photoelectron flux
      - ``40e-6 A/m^2``
      - ``setPhotoelectronFlux(double)``
    * - Photoelectron temperature
      - ``2.0 eV``
      - ``setPhotoelectronTemperature(double)``
    * - Secondary-electron temperature
      - ``5.0 eV``
      - ``setSecondaryElectronTemperature(double)``
    * - Backscatter-electron temperature
      - ``5.0 eV``
      - ``setBackscatterElectronTemperature(double)``
    * - Beam-electron temperature
      - ``20.0 eV``
      - ``setBeamElectronTemperature(double)``
    * - Electron gun defaults (per spacecraft)
      - ``currentEB=0.0 A, energyEB=0.0 eV, alphaEB=1.0``
      - via linked ``ElectronBeamMsgPayload`` input
    * - Yield tables (electron, ion, backscatter)
      - empty vectors at construction
      - ``setYieldSEEelectron``, ``setYieldSEEion``, ``setYieldBackscattered``

.. code-block:: python

    # Optional per-spacecraft sunlit-area defaults [m^2]
    chg.setSunlitAreaDefault(0, 12.0)
    chg.setSunlitAreaDefault(1, 10.0)

    # Optional fallback used only if no message and no per-spacecraft default is provided
    chg.setSunlitAreaFallback(0.0)

    # Optional bisection brackets [V]
    chg.setRootSolveBounds(-4.0e5, 4.0e5)

    # Optional environmental model constants (defaults shown)
    chg.setPhotoelectronFlux(40e-6)
    chg.setPhotoelectronTemperature(2.0)
    chg.setSecondaryElectronTemperature(5.0)
    chg.setBackscatterElectronTemperature(5.0)
    chg.setBeamElectronTemperature(20.0)

    # Optional trapezoidal integration resolution
    chg.setTrapzBins(1000)

Logging output voltages
~~~~~~~~~~~~~~~~~~~~~~~
.. code-block:: python

    servicerVoltRec = chg.voltOutMsgs[0].recorder()
    targetVoltRec   = chg.voltOutMsgs[1].recorder()

    sim.AddModelToTask(task.Name, servicerVoltRec)
    sim.AddModelToTask(task.Name, targetVoltRec)

Logging output current components
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
.. code-block:: python

    servicerCurrRec = chg.currentsOutMsgs[0].recorder()
    targetCurrRec   = chg.currentsOutMsgs[1].recorder()

    sim.AddModelToTask(task.Name, servicerCurrRec)
    sim.AddModelToTask(task.Name, targetCurrRec)

Running the simulation
~~~~~~~~~~~~~~~~~~~~~~
.. code-block:: python

    sim.InitializeSimulation()
    sim.ExecuteSimulation()

References
----------
.. [Hammerl2024] Hammerl, J., and Schaub, H., “Coupled Spacecraft Charging Due to Continuous Electron Beam Emission and Impact,” 2024.
   (Equation numbers referenced in this document correspond to that paper.)

.. [Lai2011] Lai, S. T., *Fundamentals of Spacecraft Charging*, Princeton Univ. Press, Princeton, NJ, 2011.

.. [Hippler2001] Hippler, R., Pfau, S., Schmidt, M., and Schoenbach, K. H., *Low Temperature Plasma Physics: Fundamental Aspects and Applications*,
   Wiley, Hoboken, NJ, 2001.

.. [DavisMandell2016] Davis, V. A., and Mandell, M. J., “NASCAP-2K Version 4.3 Scientific Documentation,”
   AFRL-RV-PS-TR-2017-0001, 2016.

.. [DraineSalpeter1979] Draine, B. T., and Salpeter, E. E., “On the Physics of Dust Grains in Hot Gas,”
   *Astrophysical Journal*, Vol. 231, 1979, pp. 77–94.

.. [RomeroCalvo2022] Romero-Calvo, Á., Cano-Gómez, G., and Schaub, H.,
   “Simulation and Uncertainty Quantification of Electron Beams in Active Spacecraft Charging Scenarios,” 2022.
