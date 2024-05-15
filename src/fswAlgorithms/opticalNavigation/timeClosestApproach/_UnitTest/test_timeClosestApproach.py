#
#  ISC License
#
#  Copyright (c) 2024,  University of Colorado at Boulder
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
import os

import numpy as np
import pytest
from Basilisk import __path__
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import timeClosestApproach
from Basilisk.utilities import SimulationBaseClass, macros

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


@pytest.mark.parametrize("position", [[-5e7, 7.5e6, 5e5], [-5e6, 7e6, 4e5]])  # m
@pytest.mark.parametrize("velocity", [[2e4, 0, 0], [1e4, 1e3, 2e2]])  # m/s
@pytest.mark.parametrize("filter_covariance", [np.eye(6),  np.ones([6, 6])])
def test_TimeClosestApproach(show_plots, position, velocity, filter_covariance):

    unit_task_name = "unitTask"               # arbitrary name (don't change)
    unit_process_name = "test_processes"         # arbitrary name (don't change)

    # Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    test_process_rate = macros.sec2nano(0.1)     # update process rate update time
    test_process = unit_test_sim.CreateNewProcess(unit_process_name)
    test_process.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # setup TimeClosestApproach guidance module
    tca_module = timeClosestApproach.TimeClosestApproach()
    tca_module.ModelTag = "TimeClosestApproach"
    unit_test_sim.AddModelToTask(unit_task_name, tca_module)

    state_vector = np.zeros(6)
    state_vector[:3] = position
    state_vector[3:] = velocity

    # Create the input messages.
    input_data = messaging.FilterMsgPayload()
    input_data.state = state_vector.tolist()
    input_data.covar = filter_covariance.flatten().tolist()
    filter_in_msg = messaging.FilterMsg().write(input_data)
    tca_module.filterInMsg.subscribeTo(filter_in_msg)

    tca_module.filterInMsg.subscribeTo(filter_in_msg)

    # Output messages.
    data_log_tca = tca_module.tcaOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_log_tca)

    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(test_process_rate)
    unit_test_sim.ExecuteSimulation()

    # tca_module outputs
    tca_tca = data_log_tca.timeClosestApproach
    sigmatca_tca = data_log_tca.standardDeviation


    # Expected
    tca, tca_covariance = time_of_closest_approach_calculation(position, velocity, filter_covariance)

    # make sure module output data is correct
    tolerance = 1e-10
    np.testing.assert_allclose(tca_tca,
                               tca,
                               rtol=0,
                               atol=tolerance,
                               err_msg='Variable: tca',
                               verbose=True)

    np.testing.assert_allclose(sigmatca_tca,
                               tca_covariance,
                               rtol=0,
                               atol=tolerance,
                               err_msg='Variable: tca_covariance',
                               verbose=True)


def time_of_closest_approach_calculation(r, v, filter_covariance):
    norm_v = np.linalg.norm(v)
    norm_r = np.linalg.norm(r)
    v_hat = v / norm_v
    r_hat = r / norm_r
    theta = np.arccos(np.dot(-r_hat, v_hat))
    flight_path_angle = theta - np.pi/2
    ratio = norm_v/ norm_r

    tca = np.cos(theta) / ratio

    covariance_map_to_tca = np.zeros(6)
    covariance_map_to_tca[0:3] = v_hat/norm_r
    covariance_map_to_tca[3:6] = 1/norm_v * (r_hat - np.sin(flight_path_angle) * v_hat)
    tca_covariance = (1 / ratio**2) * np.dot(covariance_map_to_tca,  np.dot(filter_covariance, covariance_map_to_tca.transpose()))

    return tca, np.sqrt(tca_covariance)


if __name__ == "__main__":
    test_TimeClosestApproach(False,
                             np.array([-5e7, 7.5e6, 5e5]),
                             np.array([2e4, 0, 0]),
                             np.eye(6)
                             )
