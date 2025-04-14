.. _mujocoDynObject:

Multi-body Dynamics with MuJoCo
===============================

.. warning::

    :beta:`Mujoco Support` The following feature is a work in progress! Extensive validation has not been performed.
    Some features might be missing, and APIs are subject to change between releases.


``DynamicObject`` subclasses are Basilisk modules that require the integration of
ordinate differential equations (ODEs) to propagate their internal state (see :ref:`bskPrinciples-9`).
The :ref:`spacecraft` module is the most commonly used ``DynamicObject`` in Basilisk, and is ideal to
represent simple or hub-centric vehicles. For vehicles or sets of vehicles with a more complex topology,
:ref:`MJScene<MJScene>` represents an alternative ``DynamicObject`` designed to facilitate the analysis of
multi-body system by leveraging the powerful capabilities of the `MuJoCo <https://mujoco.org>`_ library.

This page covers the necessary installation steps to enable :ref:`MJScene<MJScene>`, as well as a quick introduction
to important concepts in simulations with :ref:`MJScene<MJScene>`. Once the user is familiar with these, they should review the examples
presented in the *Multi-Body Dynamics Simulations with MuJoCo* section in :ref:`examples`. The user is
also encouraged to read the `MuJoCo Documentation <https://mujoco.readthedocs.io>`_, as the capabilities
exposed through :ref:`MJScene<MJScene>` are heavily based on the mathematical and design principles of MuJoCo.

Installation
------------
By default, Basilisk will not build MuJoCo-related capabilities. To enable these, modify the call to
``conanfile.py`` during installation::

    (venv) $ python conanfile.py --mujoco True --mujocoReplay True

Only the ``--mujoco True`` option is required. ``--mujocoReplay True`` will additionally build an
independent tool that can be used to visualize the results of :ref:`MJScene<MJScene>` simulations. When
using ``--mujocoReplay True``, you'll be able to call:

.. code-block:: python

    mujoco.visualize(...)

Note that using ``--mujocoReplay True`` might require the installation of other graphic libraries
in your system.

The Multi-body System
---------------------
In this documentation, we define a "multi-body dynamical system" as a collection of interconnected rigid
or flexible **bodies** whose motion is governed by **forces**, **constraints**, and kinematic relationships
(such as **joints**). This generic description reveals the breadth of systems that can be treated as multi-body
systems. For example:

#. A small-sat with deployable solar-panels and reaction wheels.
#. A constellation of satellites with gimballed antennas.
#. A multi-modular space station with a robotic arm.

Our description of multi-body systems also introduces a series of useful concepts, namely **bodies**, **joints**,
**forces**, and **constraints**.

Bodies and Joints
^^^^^^^^^^^^^^^^^
In a multi-body dynamical system, bodies are rigid or flexible components that move relative to one another, often
representing physical structures like vehicle parts or robot links. Bodies have inertial properties (mass, center
of mass, and inertia tensor) and usually a 3D geometry, which impacts their visualization and contact dynamics.
Only rigid body modeling is supported in Basilisk at this time.

Joints are the connections between bodies that
constrain their relative motion, allowing specific types of movement such as rotation (e.g., hinges),
translation (e.g., sliders), or free rotation (e.g. spherical joint). Alternatively, one may see joints as *introducing*
degrees of freedom between bodies which are otherwise fixed with respect to each other.
For example, if there are no joints between two bodies, then no degrees of freedom
are allowed, and these bodies are *welded* to each other. Alternatively, if two perpendicular translational joints
are defined between two bodies, then these bodies will be able to move freely in a plane with respect to each other.
When two bodies are joined by a spherical joint and three perpendicular translational joints, then we can say that these
two bodies move completely freely with respect to each other.

The first step in simulating a real system through :ref:`MJScene<MJScene>` is thus to break it down into individual
rigid bodies and to determine the degrees of freedom between these bodies. An important aspect of the dynamics
solver used in :ref:`MJScene<MJScene>` (as well as in many multi- body dynamic engines), is that bodies must be "organized" following a
tree topology. This means that bodies may have at most one parent but any number of child bodies. Moreover, a body may
only define joints between itself and its parent body.

Being able to simulate only tree-body topologies might sound unduly restricting,
but many real-world systems can be accurately modeled as body trees. Think of
your arm: your upper arm is connected to your torso (its parent body) through your shoulder (a two-degrees-of-freedom rotational joint),
your lower arm is connected to your upper arm through your elbow, your hand is
connected to your lower arm through your wrist, and each of the fingers is in itself a chain of links and rotational joints that
connect to the common hand parent body. Consider another example: a small-sat with three reaction wheels, a gimballed antenna,
and a robotic arm with 3 links. We might break this vehicle into the bodies and joints seen in the following figure:

    .. image:: /_images/static/sat_body_tree.svg
      :align: center
      :scale: 75%

Nevertheless, if a system has a closed-loop body topology, it can still be modeled in :ref:`MJScene<MJScene>`. To do so, one will have
to "break" these loops to obtain a tree topology, and then model the "broken" joints through *constraints* (see section below).

A multi-body system can have multiple bodies without parents, a fact that can be used to simulate independent vehicles.
Conceptually, we can say that all bodies without parent bodies actually have the "world body" as parent, a fictitious
body that represents the inertial frame of the simulation. These bodies are joined to the world body through "free"
joints, which allow full movement (6 degree of freedom joints). In this way, the multi-body system is represented
by a rooted tree.

For most designers, it is intuitive to think of the state of a body through its position and attitude with respect
to some inertial frame. However, in a multi-body system, this description leads to an over-determined state definition.
Consider, for example, a simple pendulum. This system has a single degree of freedom, the pendulum's angle. However,
the cartesian position of the pendulum weight is given by three scalar values. This is the difference between "minimal"
and "maximal" coordinate systems. MuJoCo, and thus :ref:`MJScene<MJScene>`, always uses a minimal coordinate system, where
the state of the system is given by the state of each joint (remember that we think of joints as introducers of degrees
of freedom). For convinience, this minimal coordinate state is then transformed into cartesian positions, velocities,
attitudes, and angular rates at *sites* of interest through a process known as forward kinematics.

Sites
^^^^^
Sites are reference frames of interest for simulation. They are rigidly attached at a body and have a specific position and
orientation relative to said body. In Basilisk, each body has at least two sites associated with it: its *origin* and its
*center of mass*. The origin defines the reference frame in which other sites, child bodies, and any other body element
are defined. Its location is arbitrary, but defined by users when they define the multi-body system. The center of mass site
defines the location of the center of mass, which may change in time. Other sites can be defined by users by
providing a transformation (translation and orientation) with respect to the origin site.

Users can query the position, attitude, velocity, and angular rate (with respect to the inertial frame)
of each site in the multi-body system. This can be used, for example, to easily get the position and pointing direction
of a sensor located at the end of a robotic arm. Alternatively, the information from multiple sites could be used
to measure and then control the relative velocity of two spacecraft.

The second main use of sites is to be the reference frames in which forces and torques are defined. In :ref:`MJScene<MJScene>`,
forces and torques are defined to be acting at a site's frame center and are given in said reference frame. This generally
simplifies the implementation of actuators. To model a fixed thruster, for example, we may define a site whose frame is
positioned at the location of the thruster and its Z-axis is along the thruster direction. Then, during simulation,
we may simply tell the corresponding actuator to produce a force along the Z-axis of this site, and the dynamics engine
will compute the correct force and torque vetors to apply to the body.

Actuators
^^^^^^^^^
Actuators are how the simulation framework determines what forces and torques to apply to the multi-body system.
For :ref:`MJScene<MJScene>`, we can divide actuators in two groups: those that apply a force or torque at a joint, and
those that apply a force or torque at a site.

Actuators that apply force/torques at joints can be used to model, for example, a motor that applies a torque
on a hinge or a force on a slider joint. These actuators are always of the type ``MJSingleActuator``, where
the word ``Single`` refers to the fact that a single scalar value is used to define the magnitude of the
force or torque to be applied.

Actuators that apply force/torques at sites can be used to model arbitrary external disturbances on
the system. For example, gravity is modeled as such a force/torque, and so is solar radiation pressure,
thrusters, aerodynamic forces, etc. An ``MJForceTorqueActuator`` can be used to define arbitrary
force and torque vectors on a site's reference frame. Alternatively, ``MJForceActuator`` and
``MJTorqueActuator`` can be used when only a force or torque vectors, respectively, must be modeled.
Finally, ``MJSingleActuator`` may also be used to apply a force and/or torque at a site. In this
case, the force and torque directions are fixed at multi-body-model definition time, while the magnitude
is allowed to vary during simulation. This restricted actuator model can be surprisingly useful: a fixed thruster, for
example, or the solar radiation pressure acting on a panel, can both be modeled as forces with a fixed direction
in a local reference frame.

Constraints
^^^^^^^^^^^
In MuJoCo (and thus :ref:`MJScene<MJScene>`), constraints are mathematical conditions that restrict the movement of
bodies in a simulation. MuJoCo's internal solver will effectively compute the required forces and torques to make
the system comply with these contraints. This means that this is a "soft" constraint system, where
constraints are not guaranteed to be met at every time step. The MuJoCo constraint system can be applied
to model a wide array of systems, but in this section we highlight only a few:

#. Closed-loop body topologies: In a previous section, we established that only tree body topologies can be expressed through the joint system. `Equality constraints <https://mujoco.readthedocs.io/en/stable/computation/index.html#equality>`_ must be used to model closed-loop topologies.
#. Attachments between bodies that may be active or inactive during the simulation, for example to model a rendez-vous scenario.
#. "Constrained" joints: These are scalar joints (hinges or sliders) that we want to force to have a specific position. See "Free, Constrained, and Prescribed Joints" section below.
#. Contact dynamics and friction, which are implemented through the constraint system.
#. Joint limits, also implemented as constraints.

Free, Constrained, and Prescribed Joints
----------------------------------------
By default, all joints in the multi-body system are allowed to move freely within their allowed degrees of freedom.
A hinge joint, for example, allows any rotation between two bodies along an axis. Similarly, a slider
joint allows any translation between bodies along an axis. Free joints can optionally be "limited", which enforces
that the relative position between bodies does not exceed some bounds. For instance, a limited slider may
only allow between 10 and 20 centimeter tranlsational displacement along an axis.

"Constrained" joints take these limitations a step further by forcing the joints to be in a specific,
user-defined position. This is useful when we can assume that a motor
is nearly perfectly capable of moving the system with the desired motion profile, and we are not interested
in simulating the specifics of its control system. In this sense, we offload the calculation of the necessary
forces and torques to move the joint in the desired manner to the dynamic engine's solver. Because of this,
constrained joints can cause a significant slow-down of the simulation. However, because these constraints
are eventually implemented as forces and torques between the bodies, they are physically accurate.

"Prescribed" joints are those for which we want to force the joint to have a specific position, but
do not want to bother with the dynamics that would make this possible through forces or torques. Instead,
we tell the system to ignore the joint position and velocity calculated by the engine and replace it with
the desired value. This prescription does not come with the high computational cost of a constrained
joint, and will be even faster to simulate than an explicitelly-controled joint. However, it will not
be physically accurate. This accuracy loss may be tolerable, howwver, especially if the bodies downstream of the
prescribed joint have low (relative) mass. For example, it might be acceptable to prescribe the joint
of an antenna, since we don't expect the torque that would be applied by its control motor to have a
significant effect on the satellite's hubbody.

.. warning::

    "Prescibed" joints are not currently implemented in Basilisk!
