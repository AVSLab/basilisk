
This is an instantiation of the :ref:`dynamicObject` abstract class that is a spacecraft with :ref:`stateEffector`'s and
:ref:`dynamicEffector`'s attached to it. The spacecraftPlus allows for both translation and
rotation. :ref:`stateEffector`'s such as RWs, flexible solar panel, fuel slosh etc can be added to the spacecraft by attaching
stateEffectors. :ref:`dynamicEffector`'s such as thrusters, external force and torque, SRP etc can be added to the spacecraft
by attaching dynamicEffectors. This class performs all of this interaction between stateEffectors, dynamicEffectors and
the hub.

The module
:download:`PDF Description </../../src/simulation/dynamics/SpacecraftDynamics/_Documentation/SpacecraftPlus/Basilisk-SPACECRAFTPLUS-20170808.pdf>`
contains further information on this module's function,
how to run it, as well as testing.


















