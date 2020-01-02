
This is an instantiation of the :ref:`dynamicObject` abstract class that is a spacecraft with :ref:`stateEffector`'s and
:ref:`dynamicEffector`'s attached to it. The spacecraftDynamics allows for both translation and
rotation. :ref:`stateEffector`'s such as RWs, flexible solar panel, fuel slosh etc can be added to the spacecraft by attaching
stateEffectors. :ref:`dynamicEffector`'s such as thrusters, external force and torque, SRP etc can be added to the spacecraft
by attaching dynamicEffectors. This class performs all of this interaction between stateEffectors, dynamicEffectors and
the hub.  In contrast to :ref:`spacecraftPlus`, this class allows for several complex spacecraft components to form a system.  This hubs can be rigidly connected or free-flying.

The module
:download:`PDF Description </../../src/simulation/dynamics/SpacecraftDynamics/_Documentation/SpacecraftDynamics/Basilisk-SPACECRAFTDYNAMICS-20180712.pdf>`
contains further information on this module's function,
how to run it, as well as testing.


















