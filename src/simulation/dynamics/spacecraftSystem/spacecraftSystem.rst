
.. warning::
    This module allows for multiple spacecraft units (mother craft and a docked daughter craft, etc.) to be simulated as an integrated dynamical system.  See `Dr. Cody Allard's dissertation <http://hanspeterschaub.info/Papers/grads/CodyAllard.pdf>`__ for more information.  However, this is still work in progress and not all effectors are compatible with this manner of doing the dynamics.  Use :ref:`spacecraft` to create a spacecraft simulation object unless you are familiar what this expanded spacecraft dynamics module provides.
    
Executive Summary
-----------------

This is an instantiation of the :ref:`dynamicObject` abstract class that is a spacecraft with :ref:`stateEffector`'s and
:ref:`dynamicEffector`'s attached to it. The spacecraftDynamics allows for both translation and
rotation. :ref:`stateEffector`'s such as RWs, flexible solar panel, fuel slosh etc can be added to the spacecraft by attaching
stateEffectors. :ref:`dynamicEffector`'s such as thrusters, external force and torque, SRP etc can be added to the spacecraft
by attaching dynamicEffectors. This class performs all of this interaction between stateEffectors, dynamicEffectors and
the hub.  In contrast to :ref:`spacecraft`, this class allows for several complex spacecraft components to form a system.  This hubs can be rigidly connected or free-flying.

The module
:download:`PDF Description </../../src/simulation/dynamics/spacecraftSystem/_Documentation/SpacecraftSystem/Basilisk-SPACECRAFTSYSTEM-20180712.pdf>`
contains further information on this module's function,
how to run it, as well as testing.



Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg connection is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - scStateOutMsg
      - :ref:`SCStatesMsgPayload`
      - spacecraft element state output message
    * - scMassStateOutMsg
      - :ref:`SCMassPropsMsgPayload`
      - spacecraft element mass property output message
    * - scEnergyMomentumOutMsg
      - :ref:`SCEnergyMomentumMsgPayload`
      - spacecraft element energy and momentum output message



