#
#  ISC License
#
#  Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
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

import inspect
import numpy as np
import os
import pytest
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import ephemDifferenceWithUncertainty
from Basilisk.utilities import SimulationBaseClass, unitTestSupport, macros

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))


@pytest.mark.parametrize("r_1_N, r_2_N, v_1_N, v_2_N, sig_1_N, sig_2_N",
                         [([100., 50., -20.], [-80., 20., -30.], [70., -40., 10.], [10., 90., -60.], 3., 2.),
                          ([10., -30., -10.], [70., 90., 50.], [-70., 60., 20.], [-10., 100., -40.], 5., 10.),
                          ([10., 20., 30.], [40., 50., 60.], [70., 80., 90.], [100., 110., 120.], 2., 1.),
                          ([0., 20., -30.], [40., 0., 60.], [-70., -80., 0.], [0., 110., 120.], 5., 5.),
                          ])
def test_ephem_difference_with_uncertainty(show_plots, r_1_N, r_2_N, v_1_N, v_2_N, sig_1_N, sig_2_N):
    ephem_difference_with_uncertainty_test_function(show_plots, r_1_N, r_2_N, v_1_N, v_2_N, sig_1_N, sig_2_N)


def ephem_difference_with_uncertainty_test_function(show_plots, r_1_N, r_2_N, v_1_N, v_2_N, sig_1_N, sig_2_N):
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"
    unitTestSim = SimulationBaseClass.SimBaseClass()

    testProcessRate = macros.sec2nano(0.5)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    covar_1 = np.identity(6) * sig_1_N**2
    covar_1[0, 2] = 0.5123  # populate off-diagonal elements to make sure Matrix is accurately reshaped from 6x6 to 36x1
    covar_1[4, 1] = 0.8743
    covar_2 = np.identity(6) * sig_2_N**2
    covar_2[1, 5] = 0.2384
    covar_2[3, 4] = 0.1231
    module = ephemDifferenceWithUncertainty.EphemDifferenceWithUncertainty()
    module.setCovarianceBase(covar_1)
    module.setCovarianceSecondary(covar_2)
    unitTestSim.AddModelToTask(unitTaskName, module)

    # Create the input messages
    inputEphem1 = messaging.EphemerisMsgPayload()
    inputEphem2 = messaging.EphemerisMsgPayload()

    # Set ephemeris message of object
    inputEphem1.r_BdyZero_N = np.array(r_1_N)
    inputEphem1.v_BdyZero_N = np.array(v_1_N)
    inputEphem1.timeTag = 12.345
    ephem1InMsg = messaging.EphemerisMsg().write(inputEphem1)
    module.ephemBaseInMsg.subscribeTo(ephem1InMsg)
    # Set ephemeris message of spacecraft
    inputEphem2.r_BdyZero_N = np.array(r_2_N)
    inputEphem2.v_BdyZero_N = np.array(v_2_N)
    inputEphem2.timeTag = 56.789
    ephem2InMsg = messaging.EphemerisMsg().write(inputEphem2)
    module.ephemSecondaryInMsg.subscribeTo(ephem2InMsg)

    dataLogNavTrans = module.navTransOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLogNavTrans)
    dataLogFilter = module.filterOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLogFilter)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(testProcessRate)
    unitTestSim.ExecuteSimulation()

    # Truth Values
    num_states = 6
    r_21_N_true = np.array(r_2_N) - np.array(r_1_N)
    v_21_N_true = np.array(v_2_N) - np.array(v_1_N)
    state_21_N_true = np.concatenate((r_21_N_true, v_21_N_true))
    covar_21_N_true = covar_1 + covar_2
    timeTag_true = inputEphem2.timeTag

    # module output
    r_21_N_module = dataLogNavTrans.r_BN_N[0]
    v_21_N_module = dataLogNavTrans.v_BN_N[0]
    timeTag_Nav_module = dataLogNavTrans.timeTag[0]
    state_21_N_module = dataLogFilter.state[0, :num_states]
    covar_21_N_module = dataLogFilter.covar[0, :num_states*num_states].reshape((num_states, num_states))
    timeTag_Filter_module = dataLogFilter.timeTag[0]

    # make sure module output data is correct
    tolerance = 1e-10
    np.testing.assert_allclose(r_21_N_module,
                               r_21_N_true,
                               rtol=0,
                               atol=tolerance,
                               err_msg='Variable: r_21_N',
                               verbose=True)
    np.testing.assert_allclose(v_21_N_module,
                               v_21_N_true,
                               rtol=0,
                               atol=tolerance,
                               err_msg='Variable: v_21_N',
                               verbose=True)
    np.testing.assert_allclose(timeTag_Nav_module,
                               timeTag_true,
                               rtol=0,
                               atol=tolerance,
                               err_msg='Variable: timeTag_Nav',
                               verbose=True)
    np.testing.assert_allclose(state_21_N_module,
                               state_21_N_true,
                               rtol=0,
                               atol=tolerance,
                               err_msg='Variable: state_21_N',
                               verbose=True)
    np.testing.assert_allclose(covar_21_N_module,
                               covar_21_N_true,
                               rtol=0,
                               atol=tolerance,
                               err_msg='Variable: covar_21_N',
                               verbose=True)
    np.testing.assert_allclose(timeTag_Filter_module,
                               timeTag_true,
                               rtol=0,
                               atol=tolerance,
                               err_msg='Variable: timeTag_Filter',
                               verbose=True)


if __name__ == '__main__':
    test_ephem_difference_with_uncertainty(False,
                                           [100., 50., -20.], [-80., 20., -30.],
                                           [70., -40., 10.], [10., 90., -60.],
                                           3., 2.)
