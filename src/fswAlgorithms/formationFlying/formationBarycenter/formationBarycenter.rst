Executive Summary
-----------------
This module computes a formation reference using either a Cartesian mass-weighted average of position and velocity or a mass-weighted average of orbital elements.
The default orbital-element choice retains classical-element averaging. Explicit equinoctial choices support circular and equatorial elliptic orbits without averaging undefined classical angles separately.
All choices output two navigation messages that describe the reference position and velocity. The output messages contain the same
information, although one is a C++ message and the other is a C-wrapped message.

Message Connection Descriptions
-------------------------------
The following diagram and table list all the module input and output messages.  The module message connection is
set by the user from Python.  The message type contains a link to the message structure definition, while the
description provides information on what this message is used for.

.. bsk-module-io:: formationBarycenter
    :caption: Module I/O Messages

    input scNavInMsgs NavTransMsgPayload
        vector of spacecraft navigation input messages. These are set through ``addSpacecraftToModel()``
    input scPayloadInMsgs VehicleConfigMsgPayload
        vector of spacecraft payload input messages. These are set through ``addSpacecraftToModel()``
    output transOutMsg NavTransMsgPayload
        barycenter information C++ output message
    output transOutMsgC NavTransMsgPayload
        barycenter information C-wrapped output message

Detailed Module Description
---------------------------

This module computes the barycenter of a swarm of spacecraft. For the cartesian method, a simple center of mass calculation is made for the position and velocity vectors.
Let :math:`\textbf{x}` represent either the position or the velocity vectors. The corresponding weighted average is:

.. math::
    \bar{\textbf{x}} = \dfrac{1}{m_{total}}\sum_{i}m_i\textbf{x}_i,

where :math:`m_{total}=\sum_{i}m_i`.

Classical elements
~~~~~~~~~~~~~~~~~~

For the default orbital-element averaging, the process is similar. However, the position and velocity vectors of each spacecraft must first be converted to orbital elements. Once
that is done, we take the average of each orbital element :math:`oe` as such:

.. math::
    \bar{oe} = \dfrac{1}{m_{total}}\sum_{i}m_ioe_i

This formula is only valid for semi-major axis (a), eccentricity (e) and inclination (i). For the other angular orbital elements, a problem with angle wrapping can occur
when the angles are close to zero. For example, if two spacecraft of equal mass have a true anomaly of 10 and 350 degrees, the previous averaging formula would suggest
that the mean should be 180 degrees, when in fact it should be 0. To solve this problem, a different formula is used for RAAN (:math:`\Omega`), AoP (:math:`\omega`)
and true anomaly (f):

.. math::
    \bar{\alpha} = \texttt{atan2}\left(\sum_{i}m_i\sin\alpha_i, \sum_{i}m_i\cos\alpha_i\right)

where :math:`\bar{\alpha}` is the averaged angular orbital element. The set of :math:`\bar{oe}` are then converted back into position and velocity vectors.

As stated before, the module outputs both a C++ and C-wrapped navigation messages. Both contain the same payload.

Equinoctial elements
~~~~~~~~~~~~~~~~~~~~

The opt-in choices average one common equinoctial representation for all spacecraft. Let :math:`I=+1` for ``EQUINOCTIAL_PROGRADE`` and :math:`I=-1` for ``EQUINOCTIAL_RETROGRADE``. The element definitions are

.. math::

    P_1 &= e\sin(\omega+I\Omega), & P_2 &= e\cos(\omega+I\Omega), \\
    Q_1 &= \tan(i/2)^I\sin\Omega, & Q_2 &= \tan(i/2)^I\cos\Omega, \\
    l &= M+\omega+I\Omega.

The semimajor axis :math:`a`, :math:`P_1`, :math:`P_2`, :math:`Q_1` and :math:`Q_2` use the linear mass-weighted average. Mean longitude :math:`l` uses the circular average shown above. The module calculates the elements directly from position and velocity and reconstructs the averaged state in the equinoctial plane; it does not recover and average singular classical angles.

Mean longitude retains the shared two-body mean motion when all members have the same semimajor axis and the circular mean is defined. For unequal semimajor axes, the output velocity is the Keplerian velocity of the averaged instantaneous orbit; it need not equal the time derivative of a reference recomputed from the whole formation. An orbital-element average is generally different from the physical Cartesian center of mass.

The same value of :math:`I` is used throughout one average. The module does not automatically change representations near an inclination boundary. ``EQUINOCTIAL_RETROGRADE`` covers :math:`i=\pi`, where the prograde representation is singular, but has its own singularity at :math:`i=0`. These choices describe the same individual orbit where both are defined, but averaging different representations need not produce the same formation reference.

Model Assumptions and Limitations
---------------------------------

This code makes the following assumptions:

- **Gravitational parameter is known**

This code has the following limitations:

- **Legacy classical singularities**: the unchanged classical-element method is unreliable near circular or equatorial orbits, where some separately averaged angles are undefined. Select a suitable equinoctial choice explicitly for these cases.
- **Equinoctial orbit range**: only finite, nonradial elliptic states with positive semimajor axes and :math:`0\le e<1` are supported by the new choices. Parabolic (:math:`e=1`), hyperbolic (:math:`e>1`) and radial states raise a Basilisk error. Binding energy indistinguishable from zero at floating-point precision is also rejected. The legacy modes are not given new orbit-domain restrictions.
- **Excluded pole**: the prograde choice excludes the retrograde equatorial pole and vice versa. States at the excluded pole, or within eight machine epsilons in the normalized transverse angular momentum there, raise a Basilisk error. Select the other representation for a formation near that pole.
- **Weights**: the equinoctial choices require finite nonnegative masses and a finite positive total mass. Zero-weight spacecraft do not contribute.
- **Undefined longitude**: if the magnitude of the normalized weighted sine-and-cosine sum is at most 64 machine epsilons, the circular-mean longitude is numerically undefined and raises a Basilisk error. Equal masses on opposite orbital phases are one such case. There is no automatic Cartesian or classical fallback.
- **Gravity and numerical range**: equinoctial averaging requires a finite positive gravitational parameter. Unrepresentable intermediate elements or output states raise a Basilisk error. Accuracy degrades near the parabolic limit; the new choices do not promise reliable representation of arbitrarily extreme ellipses.


User Guide
----------

This section contains conceptual overviews of the code and clear examples for the prospective user.

Module Setup
~~~~~~~~~~~~

The module is created in Python using:

.. code-block:: python
    :linenos:

    barycenterModule = formationBarycenter.FormationBarycenter()
    barycenterModule.ModelTag = 'barycenter'

A sample setup is done using:

.. code-block:: python
    :linenos:

    # Configure spacecraft state input messages
    scNavMsgData1 = messaging.NavTransMsgPayload()
    scNavMsgData1.r_BN_N = rN1
    scNavMsgData1.v_BN_N = vN1
    scNavMsg1 = messaging.NavTransMsg().write(scNavMsgData1)

    scNavMsgData2 = messaging.NavTransMsgPayload()
    scNavMsgData2.r_BN_N = rN2
    scNavMsgData2.v_BN_N = vN2
    scNavMsg2 = messaging.NavTransMsg().write(scNavMsgData2)

    # Configure spacecraft mass input messages
    scPayloadMsgData1 = messaging.VehicleConfigMsgPayload()
    scPayloadMsgData1.massSC = 100
    scPayloadMsg1 = messaging.VehicleConfigMsg().write(scPayloadMsgData1)

    scPayloadMsgData2 = messaging.VehicleConfigMsgPayload()
    scPayloadMsgData2.massSC = 150
    scPayloadMsg2 = messaging.VehicleConfigMsg().write(scPayloadMsgData2)

    # add spacecraft input messages to module
    barycenterModule.addSpacecraftToModel(scNavMsg1, scPayloadMsg1)
    barycenterModule.addSpacecraftToModel(scNavMsg2, scPayloadMsg2)

No further setup is needed for the cartesian method. If the user wants to use orbital elements, the following additional code is needed:

.. code-block:: python
    :linenos:

    barycenterModule.useOrbitalElements = True
    barycenterModule.mu = mu

This retains classical averaging because ``orbitalElementType`` defaults to ``CLASSICAL``. To opt in to mean-longitude equinoctial averaging, also set:

.. code-block:: python

    barycenterModule.orbitalElementType = barycenterModule.EQUINOCTIAL_PROGRADE

For a formation near the retrograde equator, select the common retrograde representation instead:

.. code-block:: python

    barycenterModule.orbitalElementType = barycenterModule.EQUINOCTIAL_RETROGRADE

When ``useOrbitalElements`` is false, ``orbitalElementType`` is ignored and the existing Cartesian calculation is unchanged.
