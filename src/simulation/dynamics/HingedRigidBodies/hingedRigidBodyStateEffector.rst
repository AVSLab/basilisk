
Executive Summary
-----------------

The hinged rigid body class is an instantiation of the state effector abstract class. The integrated test is validating the interaction between the hinged rigid body module and the rigid body hub that it is attached to. In this case, a hinged rigid body has an inertia tensor and is attached to the hub by a single degree of freedom torsional hinged with a linear spring constant and linear damping term.



Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg variable name is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.


.. bsk-module-io:: hingedRigidBodyStateEffector
    :caption: Module I/O Messages

    input hingedRigidBodyInMsg HingedRigidBodyMsgPayload
        (Optional) Input message of the reference angle and angle rate.
    input motorTorqueInMsg ArrayMotorTorqueMsgPayload
        (Optional) Input message of the hinge motor torque value.
    output hingedRigidBodyOutMsg HingedRigidBodyMsgPayload
        Output message containing the panel hinge state angle and angle rate.
    output hingedRigidBodyConfigLogOutMsg SCStatesMsgPayload
        Output message containing the panel inertial position and attitude states.


Initialization and Reset
------------------------
``mass`` must be finite and non-negative. A zero-mass body remains supported when its
configured rotational inertia permits nonsingular hinge dynamics. ``dcm_HB`` must be a
finite, orthogonal, right-handed rotation matrix; scaled axes and reflections are rejected.

These checks run before state registration during spacecraft initialization, including when
the effector is attached without being added to a task. ``Reset()`` performs the same checks
without accessing parent states or changing integrated hinge states, motor commands, or
reference values. Invalid configurations raise ``BasiliskError``. See :ref:`effectorInitialization`.


Detailed Module Description
---------------------------

Mathematical Modeling
^^^^^^^^^^^^^^^^^^^^^
See
Allard, Schaub, and Piggott paper: `General Hinged Solar Panel Dynamics Approximating First-Order Spacecraft Flexing <http://dx.doi.org/10.2514/1.A34125>`__
for a detailed description of this model. A hinged rigid body has 2 states: theta and thetaDot.

For additional information about connecting a reference, see Bascom and Schaub paper: `Modular Dynamic Modeling of Hinged Solar Panel Deployments <https://hanspeterschaub.info/Papers/Bascom2022.pdf>`__

The module
:download:`PDF Description </../../src/simulation/dynamics/HingedRigidBodies/_Documentation/Basilisk-HINGEDRIGIDBODYSTATEEFFECTOR-20170703.pdf>`
contains further information on this module's function,
how to run it, as well as testing.

.. note::

    In contrast to :ref:`spinningBodyOneDOFStateEffector`, this module assumes:

    - rigid body inertia matrix is diagonal as seen in the hinged body :math:`\cal S` frame
    - the center of mass lies on the :math:`\hat{\bf s}_1` axis

Module Testing
^^^^^^^^^^^^^^
Seven dynamics scenarios use the regular :ref:`spacecraft` module with two hinged panels.
The five cases in :ref:`test_hingedRigidBodyDynamics` cover conservation and analytical
response calculations:

#. **Gravity without damping:** orbital energy, orbital angular momentum, rotational
   energy, and rotational angular momentum remain constant.
#. **Free flight without damping:** the same four quantities remain constant.
#. **Free flight with damping:** orbital energy and both angular momenta remain constant.
   Rotational energy decreases, and its loss agrees with the integrated hinge damping power.
#. **Steady-state deflection:** under constant force, both damped hinge angles approach
   the nonlinear static torque-balance solution. Angles and rates are checked throughout
   the final settling window.
#. **Frequency and amplitude:** a constant force is applied and then removed. Periods
   and peak deflections are measured directly from both Basilisk hinge output messages
   during the forced and free phases and compared with an independent small-angle solution.
   Every maximum and minimum is checked, and the required number of extrema follows
   from the phase duration and analytical period. Full angle and rate histories are
   also compared, including the final partial cycle, to detect fading or stalled motion.

These tests reject incomplete recordings and non-finite hinge states. Conservation is
checked over the full recorded histories with errors normalized by the initial magnitude
and a tolerance of ``1e-10``. The damped energy loss agrees with the integrated damping
power to a relative tolerance of ``1e-6``. Steady-state angles and rates use absolute
tolerances of ``1e-6`` rad and ``1e-6`` rad/s, respectively. Frequency and amplitude
comparisons allow a relative error of 0.5% for the small-angle approximation and sampled
peak times. Full-history angle and rate errors are bounded by 0.5% of the analytical
amplitude and peak rate, respectively, so the tolerance remains meaningful at zero crossings.

For the symmetric planar force-response cases, the total spacecraft mass is :math:`M`,
each panel has mass :math:`m`, hinge-to-center-of-mass distance :math:`d`, center-of-mass
inertia :math:`I_{yy}` about an axis parallel to the hinge axis, and spring constant
:math:`k`. The static balance is

.. math::

    k\theta_{\mathrm{ss}} + m d \frac{F}{M}\cos\theta_{\mathrm{ss}} = 0.

Eliminating hub translation from the linearized equations gives

.. math::

    J_{\mathrm{eff}} = I_{yy} + m d^2 - \frac{2 (m d)^2}{M},
    \qquad
    \omega = \sqrt{\frac{k}{J_{\mathrm{eff}}}}.

Starting from rest, the forced response is
:math:`\theta(t) = \theta_{\mathrm{ss,lin}}(1-\cos\omega t)`, where
:math:`\theta_{\mathrm{ss,lin}} = -m d F/(M k)`. After thrust ends at :math:`t_{\mathrm{off}}`,
the predicted free amplitude is
:math:`\sqrt{\theta(t_{\mathrm{off}})^2 + (\dot\theta(t_{\mathrm{off}})/\omega)^2}`.
The reference values come from these equations and the prescribed force duration;
the measured periods and amplitudes come from Basilisk, not the independent Lagrangian trajectory.

The remaining two cases are in :ref:`test_hingedRigidBodyStateEffector`:

- **Motor torque:** applies a hinge motor torque and checks system rotational angular
  momentum, stationary center-of-mass position, and the initial panel configuration messages.
- **Lagrangian comparison:** compares spacecraft translation, attitude, and both hinge
  angles at selected checkpoints against an independently integrated planar model, using
  an absolute tolerance of ``1e-10`` in meters or radians as applicable.

The separate :ref:`test_hingedRigidBodyDefaultConfig` test checks the default identity
inertia tensor and hinge-frame rotation matrix. The
:download:`PDF Description </../../src/simulation/dynamics/HingedRigidBodies/_Documentation/Basilisk-HINGEDRIGIDBODYSTATEEFFECTOR-20170703.pdf>`
provides additional analytical background; the executable tests above define the current coverage.

User Guide
----------
This section is to outline the steps needed to setup a Hinged Rigid Body State Effector in python using Basilisk.

#. Import the hingedRigidBodyStateEffector class::

    from Basilisk.simulation import hingedRigidBodyStateEffector

#. Create an instantiation of a Hinged Rigid body::

    panel1 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()

#. Define all physical parameters for a Hinged Rigid Body. For example::

    IPntS_S = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]

   Do this for all of the parameters for a Hinged Rigid Body seen in the Hinged Rigid Body 1 Parameters Table.

#. (Optional) Define a unique name for each state.  If you have multiple panels, they each must have
   a unique name.  If these names are not specified, then the default names are used which are
   incremented by the effector number::

    panel1.thetaInit = 5*numpy.pi/180.0
    panel1.thetaDotInit = 0.0

#. Define a unique name for each state::

    panel1.nameOfThetaState = "hingedRigidBodyTheta1"
    panel1.nameOfThetaDotState = "hingedRigidBodyThetaDot1"

#. Define an optional motor torque input message::

    panel1.motorTorqueInMsg.subscribeTo(msg)

#. The angular states of the panel are created using an output message ``hingedRigidBodyOutMsg``.

#. The panel config log state output message is ``hingedRigidBodyConfigLogOutMsg``.

#. Add the panel to your spacecraft::

    scObject.addStateEffector(panel1)

   See :ref:`spacecraft` documentation on how to set up a spacecraft object.

#. Add the module to the task list::

    unitTestSim.AddModelToTask(unitTaskName, panel1)


Hosting a Dynamic Effector
--------------------------
This effector supports the branching described in :ref:`bskPrinciples-11`, so a compatible
dynamic effector can be carried by the panel rather than by the hub::

    panel1.addDynamicEffector(childEffector)

This effector then makes its inertial position, velocity, attitude, and angular velocity available
in place of the hub's, and the child reads whichever of the four its model needs. Any geometry given
to the child is expressed in that panel's frame rather than the hub body frame. Both this effector
and the child are still added to the task in the usual way.
