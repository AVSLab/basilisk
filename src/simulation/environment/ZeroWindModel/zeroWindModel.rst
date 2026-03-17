Executive Summary
-----------------

Zero wind model representing atmospheric conditions with no wind perturbations beyond co-rotation.
The module is a sub-class of the :ref:`windBase` base class.  See that class for the nominal messages
used and general instructions.

Module Description
------------------

The ``ZeroWindModel`` computes atmospheric wind velocity due solely to planetary co-rotation. For a spacecraft at
inertial position :math:`\mathbf{r}_{BP,N}` relative to the planet center, the co-rotating atmosphere velocity
:math:`\mathbf{v}_{\mathrm{corot},N}` is:

.. math::
    \mathbf{v}_{\mathrm{corot},N} = \boldsymbol{\omega}_{P/N} \times \mathbf{r}_{BP,N}

where :math:`\boldsymbol{\omega}_{P/N}` is the planet angular velocity in the inertial frame (default: Earth rotation rate of :math:`7.292115\times 10^{-5}` rad/s).

The output :ref:`WindMsgPayload` contains:

- ``v_air_N``: Full air velocity = co-rotating velocity (since ``v_wind_N`` = 0)
- ``v_wind_N``: Wind perturbation velocity = 0 (no additional wind)

Planet angular velocity can be set manually via ``setPlanetOmega_N()`` or automatically derived from SPICE
when ``J20002Pfix_dot`` is available.

User Guide
----------

Setup
^^^^^

Import the module and create an instance:

.. code-block:: python

    from Basilisk.simulation import zeroWindModel
    windModel = zeroWindModel.ZeroWindModel()

Connect the required input messages:

.. code-block:: python

    # Connect planet state message (from SPICE interface)
    windModel.planetPosInMsg.subscribeTo(planetStateMsg)

    # Add spacecraft to track
    windModel.addSpacecraftToModel(scStateMsg)

Configuration
^^^^^^^^^^^^^

Set planet angular velocity (default: Earth rotation rate):

.. code-block:: python

    import numpy as np
    omega = np.array([0.0, 0.0, 7.292115e-5])  # [rad/s]
    windModel.setPlanetOmega_N(omega)

Enable/disable SPICE-derived angular velocity (default: enabled):

.. code-block:: python

    windModel.setUseSpiceOmegaFlag(True)  # Use SPICE when available
    windModel.setUseSpiceOmegaFlag(False) # Use only manual value

See :ref:`windBase` for details on the SPICE vs. manual omega selection logic.

Message Connection Descriptions
-------------------------------

The following table lists all the module specific input and output messages.  The module msg connection is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - planetPosInMsg
      - :ref:`SpicePlanetStateMsgPayload`
      - Planet SPICE state input message used to compute spacecraft position relative to planet and derive planet angular velocity
    * - scStateInMsgs
      - Array of :ref:`SCStatesMsgPayload`
      - Spacecraft state input messages (vector). Use ``addSpacecraftToModel()`` to add spacecraft.
    * - envOutMsgs
      - Array of :ref:`WindMsgPayload`
      - Wind velocity output messages (vector). Automatically created when spacecraft are added.
