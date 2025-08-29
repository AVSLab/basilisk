# ISC License
#
# Copyright (c) 2024, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.


#
#   Unit Test Script
#   Module Name:        spinningBody2TwoDOF
#   Author:             Andrew Morell
#   Creation Date:      December 18, 2024
#

import inspect
import os
import pytest
import numpy
import matplotlib.pyplot as plt

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('simulation')

from Basilisk.utilities import SimulationBaseClass, unitTestSupport, macros
from Basilisk.simulation import spacecraft, spinningBodyTwoDOFStateEffector, gravityEffector
from Basilisk.architecture import messaging


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_

@pytest.mark.parametrize("cmdTorque1, lock1, theta1Ref, cmdTorque2, lock2, theta2Ref", [
    (0.0, False, 0.0, 0.0, False, 0.0)
    , (0.0, True, 0.0, 0.0, False, 0.0)
    , (0.0, False, 0.0, 0.0, True, 0.0)
    , (0.0, True, 0.0, 0.0, True, 0.0)
    , (1.0, False, 0.0, -2.0, False, 0.0)
    , (0.0, False, 10.0 * macros.D2R, 0.0, False, -5.0 * macros.D2R)
    , (0.0, False, -5.0 * macros.D2R, 0.0, False, 10.0 * macros.D2R)
])
def test_spinningBody(show_plots, cmdTorque1, lock1, theta1Ref, cmdTorque2, lock2, theta2Ref):
    r"""
    **Validation Test Description**
    This integrated test checks the functionality of attaching a dynamic effector onto a spinningBodyTwoDOF such that
    spinningBodyTwoDOF is the dynamical parent rather than the hub. Note that this can also be done in some cases using
    the messaging system (see test_thruster_dynamics_attached_body.py) but is done here using the dynParamManager to
    achieve time varying motion with respect to the hub, two-way dynamical coupling, and sub-integration timestep
    resolution.

    The integrated test sets up an extForceTorque dynamic effector as normal, but then converts the direction and
    location to take into account being in the spinningBodyTwoDOF local body. The extForceTorque effector is set to
    apply a force and torque for the first half of the simulation, and then turn off.




    As with the other tests, the expected forces and torques are compared with the values from the module to check that
    everything matches accordingly.

    This unit test sets up a spacecraft with a single-axis rotating rigid body attached to a rigid hub. The spinning
    body's center of mass is off-center from the spinning axis and the position of the axis is arbitrary. The scenario
    includes gravity acting on both the spacecraft and the effector.

    **Description of Variables Being Tested**

    In this file we are checking the principles of conservation of energy and angular momentum. Both the orbital and
    rotational energy and angular momentum must be maintained when conservative forces like gravity are present.
    Therefore, the values of the variables

    - ``finalOrbAngMom``
    - ``finalOrbEnergy``
    - ``finalRotAngMom``
    - ``finalRotEnergy``

    against their initial values.
    """
    [testResults, testMessage] = spinningBody(show_plots, cmdTorque1, lock1, theta1Ref, cmdTorque2, lock2, theta2Ref)
    assert testResults < 1, testMessage
