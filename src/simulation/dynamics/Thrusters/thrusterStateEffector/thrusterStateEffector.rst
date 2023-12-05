
Executive Summary
-----------------

This module provides an implementation of the behavior of thrusters. Thrusters are modeled dynamically using a first-order ordinary differential equation, which is compatible with variable time step integrators.
With this module, there are now two different thruster implementations. See :ref:`thrusterDynamicEffector` for the previous version, which implemented thrusters using on and off-ramps, although not compatible with variable time step integrators.

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
    * - cmdsInMsg
      - :ref:`THRArrayOnTimeCmdMsgPayload`
      - (optional) input message with thruster commands. If not connected the thruster commands are set to zero.
    * - thrusterOutMsgs
      - :ref:`THROutputMsgPayload`
      - output message vector for thruster data


Detailed Module Description
---------------------------

Thruster Modeling
~~~~~~~~~~~~~~~~~
The functionalities of this thruster implementation are identical to the ones seen in :ref:`thrusterDynamicEffector`. For a general description of the thruster implementation in Basilisk, namely how the forces and torques are computed, see that module's documentation.

The main difference between the two thruster implementations comes from how the ``thrustFactor`` is computed and updated at each timestep. The ``thrustFactor`` is a parameter specific to each thruster and ranges from 0 to 1. It defines what the current magnitude of the thrust is: at 0 the thruster is off, and at 1 the thruster is at ``maxThrust``. While the previous thruster module computed the ``thrustFactor`` using on and off-ramps at the beginning and end of the thrusting maneuver, respectively, with a constant steady-state value of 1 in-between, this module updates that value through a first-order ordinary differential equation.

The state variable used in this state effector is :math:`\kappa`, which is a vector of ``thrustFactors`` for each thruster. As with :ref:`thrusterDynamicEffector`, the ``addThruster()`` method augments the vector of thruster inside the module. However, for this module the state vector :math:`\kappa` is also augmented in this method.

Let :math:`\kappa_i` correspond to the ``thrustFactor`` of the i-th thruster. It has two governing differential equations, depending on whether a thrust command is present or not. The differential equation when thrusting is given by

.. math ::

  \dot{\kappa_i} = \omega(1-\kappa_i)

where :math:`\omega`, in rad/s, corresponds to the cutoff frequency of the first-order filter, and is thruster-specific. The differential equation when not thrusting is given by

.. math ::

  \dot{\kappa_i} = -\omega\kappa_i

While the value of :math:`\kappa_i` for each thruster is computed numerically by integrating these ODEs, there are closed-form solutions to these equations that are used for validation and verification purposes. The solutions to each of these differential equations are given, respectively, by

.. math::
  \kappa_i(t) = 1 + (\kappa_{0,i}-1)e^{-\omega t}, \qquad \kappa_i(t) = \kappa_{0,i}e^{-\omega t}

where :math:`\kappa_{0,i}` corresponds to the initial conditions of the ``thrusterFactor`` variable for the i-th particular thruster.


Adding Thrusters to Auxiliary Bodies
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
This thruster effector does not have to only be attached to the primary spacecraft body :math:`B`.
Rather, it is possible to attach this to a rigid body sub-component of the spacecraft where
now the thruster location and orientations vary with time with respect to the :math:`B` frame.
However, note that currently attaching a thruster to a sub-component just determines where the
thruster location and thrust direction are.  The thrust does not impact the dynamics of a hinged
panel, for example.  Currently there is one-way coupling here where the sub-component states
impact the location and orientation of the thruster, but firing the thruster does not impact
the effector.

Let :math:`F` be the body-fixed frame of the rigid body sub-component, such as
:ref:`prescribedMotionStateEffector`.  With this effector case it is ok that the thruster
does not impact the effector as the effector motion is prescribed.
If the thruster is added to an auxiliary body :math:`F`
which is connected to the main spacecraft body :math:`B`, then this is accomplished using::

    thrusterSet.addThruster(thruster1, bodyStatesMsg)

Here ``thruster1`` is the thruster being added, and ``bodyStatesMsg`` is the state output message of
the platform :math:`F`.  In the ``Update()`` routine the position of the platform :math:`F` relative
to :math:`B` is recomputed each time step.

The thruster state output message contains variables that end with ``_B``.  These vectors are always
written with spacecraft body frame :math:`B` vector components.  However, the output message
states ``thrusterDirection`` and ``thrusterLocation`` don't contain the ``_B`` suffix because
they are written in either :math:`B` frame components if the thruster is attached to the body
frame :math:`B`, and in the sub-component frame :math:`F` if attached to an auxiliary spacecraft
component.



Model Assumptions and Limitations
---------------------------------

Assumptions
~~~~~~~~~~~

The model assumes that the behavior of a thruster is represented by a first-order filter. Therefore, due to the simplicity of the model, some real-world behaviors cannot be simulated, such as overshoot or 
damping.

The thruster module also assumes that the thruster always thrusts along its thrust directions axis. No dispersion is added to the thrust axis with respect to the nominal thruster axis.

Limitations
~~~~~~~~~~~

One of the limitations of this model relates to the dynamic nature of this thruster implementation. The thrust is simulated through the thrust factor, which is updated by integrating a differencial equation. This means that it is not possible to reproduce on-off behavior, where the thruster goes from not thrusting to being at maximum thrust or vice-versa. Using this dynamic model, we would have to use infinite derivatives to 
reproduce this behavior, which is not numerically feasible. To replicate this behavior, the user should use the older version of the thruster effector (:ref:`thrusterDynamicEffector`) with both on or off-ramps disabled.

Another limitation is that the :math:`I_{sp}` used is constant throughout the simulation. This means that the mass flow rate of the thruster is constant - the thruster will lose mass as soon as the valve is open, independent of how much thrust force is being produced. If the user needs to change the :math:`I_{sp}` value of any of the thrusters, the simulation needs to be stop and restarted.


.. note::
  The dynamic behaviour of this module is governed by the ``cutoffFrequency`` variable inside :ref:`THRSimConfig`. Its default value is equal to 10 rad/s. All variables related to on and off-ramps have no impact on this module and are instead supposed to be used to determine the dynamic behaviour within :ref:`thrusterDynamicEffector`.



User Guide
----------

This section contains conceptual overviews of the code and clear examples for the prospective user.

Module Setup
~~~~~~~~~~~~

To use the thruster state effector module, the user first needs to create the thruster and populate it with the necessary information, such as thruster magnitude, minimum on time, etc. This can be done with the help 
of the :ref:`simIncludeThruster` Basilisk Python library. The code to create a generic thruster is shown below:

.. code-block:: python

    thFactory = simIncludeThruster.thrusterFactory()
    TH1 = thFactory.create('MOOG_Monarc_1',
                           [1, 0, 0],  # location in B-frame
                           [0, 1, 0]  # thruster force direction in B-frame
                          )

The code above creates the generic thruster Monarc 1. To create the thruster effector and connect the thruster to it, the code below is used:

.. code-block:: python

    thrustersStateEffector = thrusterStateEffector.ThrusterStateEffector()
    thFactory.addToSpacecraft("Thrusters",
                              thrustersStateEffector,
                              scObject)

Assuming that the user has created a list of initial conditions called ``initialConditions``, then setting the initial conditions for all thrusters is done with the code below:

.. code-block:: python

    thrustersStateEffector.kappaInit = messaging.DoubleVector(initialConditions)
