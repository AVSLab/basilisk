#
#  ISC License
#
#  Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#


#
#   Unit Test Script
#   Module Name:        cppModuleTemplateParametrized
#   Author:             (First Name) (Last Name)
#   Creation Date:      Month Day, Year
#

import pytest
import numpy as np
from Basilisk.architecture import messaging
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport
from Basilisk.utilities import macros
from Basilisk.fswAlgorithms import foceInertialToBody


def test_unit_quaternion():

    sigma_BN = np.array([0,0,0])
    force_inertial = np.array([10.0, -0.4, -4.3])
    force_body_expected = force_inertial
    Inertia2Body_test_function(force_inertial, sigma_BN, force_body_expected)



def Inertia2Body_test_function(force_inertial, sigma_BN, force_body_expected):

    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProcessRate = macros.sec2nano(0.5)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    FoceInertialToBody = foceInertialToBody.FoceInertialToBody()
    unitTestSim.AddModelToTask(unitTaskName, FoceInertialToBody)

    cmdForceInertialInPayload = messaging.CmdForceInertialMsgPayload()
    cmdForceInertialInPayload.forceRequestInertial = force_inertial
    cmdForceInertialInMsg = messaging.CmdForceInertialMsg().write(cmdForceInertialInPayload)

    NavAttMsgInPayload = messaging.NavAttMsgPayload()
    NavAttMsgInPayload.sigma_BN = sigma_BN
    NavAttMsg = messaging.NavAttMsg().write(NavAttMsgInPayload)

    FoceInertialToBody.CmdForceInertialInMsg.subscribeTo(cmdForceInertialInMsg)
    FoceInertialToBody.NavAttInMsg.subscribeTo(NavAttMsg)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(0.5))
    unitTestSim.ExecuteSimulation()

    force_body = FoceInertialToBody.CmdForceBodyOutMsg.read().forceRequestBody
    print(force_body)
    rta = unitTestSupport.isVectorEqual(force_body, force_body_expected, 1e-3)
    assert rta == 1, f"Test failed, expected={force_body_expected}, result={force_body}"


if __name__ == "__main__":
    test_unit_quaternion()
