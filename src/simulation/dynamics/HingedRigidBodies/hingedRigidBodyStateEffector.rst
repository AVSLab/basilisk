
The hinged rigid body class is an instantiation of the state effector abstract class. The integrated test is validating the interaction between the hinged rigid body module and the rigid body hub that it is attached to. In this case, a hinged rigid body has a diagonal inertia tensor and is attached to the hub by a single degree of freedom torsional hinged with a linear spring constant and linear damping term. The integrated tests has six scenarios it is testing. The first three are: one with gravity and no damping, one without gravity and without damping, and one without gravity with damping. These first three tests are verifying energy and momentum conservation. In the first two cases orbital energy, orbital momentum, rotational energy, and rotational angular momentum should all be conserved. In the third case orbital momentum, orbital energy, and rotational momentum should be conserved. This integrated test validates for all three scenarios that all of these parameters are conserved. The fourth scenario is verifying that the steady state deflection while a constant force is being applied matches the back of the envelope (BOE) calculation. The fifth scenario applies a constant force and removes the force and the test verifies that the frequency and amplitude match the BOE calculations. And the sixth scenario verifies that Basilisk gives identical results to a planar Lagrangian dynamical system created independently.

See
Allard, Schaub, and Piggott paper: `General Hinged Solar Panel Dynamics Approximating First-Order Spacecraft Flexing <http://dx.doi.org/10.2514/1.A34125>`__
for a detailed description of this model. A hinged rigid body has 2 states: theta and thetaDot


The module
:download:`PDF Description </../../src/simulation/dynamics/HingedRigidBodies/_Documentation/Basilisk-HINGEDRIGIDBODYSTATEEFFECTOR-20170703.pdf>`
contains further information on this module's function,
how to run it, as well as testing.







