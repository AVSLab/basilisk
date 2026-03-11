
# ISC License
#
# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
#   Module Name:        facetedSpacecraftProjectedArea
#   Author:             Leah Kiner
#

import numpy as np
import pytest

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.simulation import facetedSpacecraftProjectedArea
from Basilisk.architecture import messaging


@pytest.mark.parametrize("general_heading, sun_heading, velocity_heading", [ (True, False, False),
                                                                             (False, True, False),
                                                                             (False, False, True)])
def test_facetedSpacecraftProjectedArea(show_plots,
                                        general_heading,
                                        sun_heading,
                                        velocity_heading):
    r"""
    **Verification Test Description**

    This unit test verifies that the faceted spacecraft projected area BSK module correctly computes the per-facet
    and total projected area of a faceted spacecraft model using a provided heading direction vector. The module
    can be configured for three different heading direction types. The first (most direct) option is to provide a
    general heading direction using the module ``BodyHeadingMsgPayload`` input message. In this case, the heading vector
    must be provided relative to and expressed in the hub body frame B. The second heading configuration option is for
    the sun direction heading. In this case, the facet sunlit areas are computed. This option requires two module input
    messages (``SpicePlanetStateMsgPayload`` and ``SCStatesMsgPayload``) to be configured. The spice input message
    must contain the Sun inertial state information, while the spacecraft state message contains the spacecraft hub
    inertial state information. The third heading option is to configure the velocity heading for the exposed area to
    drag, which requires setting the module ``SCStatesMsgPayload`` input message. In this case, the heading vector
    is set as the direction of the spacecraft hub's inertial velocity vector.

    This test sets up a simulation for each of the three heading configuration options. All tests set up the faceted
    spacecraft projected area BSK module identically with two different facets. The test checks that the computed
    truth values match the values output from the module.

    **Test Parameters**

    Args:
        general_heading (bool): Option 1: Direct general heading test
        sun_heading (bool): Option 2: Sun heading for sunlit area calculation test
        velocity_heading (bool): Option 3: Spacecraft inertial velocity heading for area exposed to drag test

    **Description of Variables Being Tested**

     The test checks that the module correctly computes the per-facet and total projected area of a faceted spacecraft
     model using a provided heading direction vector. The specific variables checked are the per-facet and total
     projected area(s).
    """

    task_name = "unitTask"
    process_name = "TestProcess"
    test_sim = SimulationBaseClass.SimBaseClass()
    test_time_step_sec = 1.0  # [s]
    test_process_rate = macros.sec2nano(test_time_step_sec)
    test_process = test_sim.CreateNewProcess(process_name)
    test_process.addTask(test_sim.CreateNewTask(task_name, test_process_rate))

    # Facet geometry information
    num_facets = 2
    facet_area_list = [0.5, 1.0]  # [m^2]
    facet_r_CopB_B_list = [np.array([-0.1, 0.1, -0.1]),
                           np.array([0.1, -0.1, -0.1])]  # [m]
    facet_nHat_B_list = [np.array([1.0, 0.0, 0.0]),
                         np.array([0.0, 1.0, 0.0])]
    facet_rotHat_B_list = [np.array([0.0, 1.0, 0.0]),
                           np.array([1.0, 0.0, 0.0])]
    facet_diffuse_coeff_list = [0.1, 0.1]
    facet_specular_coeff_list = [0.9, 0.9]

    # Create the facet element input messages
    facet_1_message_data = messaging.FacetElementBodyMsgPayload(
        area = facet_area_list[0],
        r_CopB_B = facet_r_CopB_B_list[0],
        nHat_B = facet_nHat_B_list[0],
        rotHat_B = facet_rotHat_B_list[0],
        c_diffuse = facet_diffuse_coeff_list[0],
        c_specular = facet_specular_coeff_list[0],
    )
    facet_2_message_data = messaging.FacetElementBodyMsgPayload(
        area = facet_area_list[1],
        r_CopB_B = facet_r_CopB_B_list[1],
        nHat_B = facet_nHat_B_list[1],
        rotHat_B = facet_rotHat_B_list[1],
        c_diffuse = facet_diffuse_coeff_list[1],
        c_specular = facet_specular_coeff_list[1],
    )
    facet_1_message = messaging.FacetElementBodyMsg().write(facet_1_message_data)
    facet_2_message = messaging.FacetElementBodyMsg().write(facet_2_message_data)

    # Create the faceted spacecraft projected area module
    faceted_sc_projected_area = facetedSpacecraftProjectedArea.FacetedSpacecraftProjectedArea()
    faceted_sc_projected_area.ModelTag = "facetedSpacecraftProjectedArea"
    faceted_sc_projected_area.setNumFacets(num_facets)
    faceted_sc_projected_area.facetElementBodyInMsgs[0].subscribeTo(facet_1_message)
    faceted_sc_projected_area.facetElementBodyInMsgs[1].subscribeTo(facet_2_message)
    test_sim.AddModelToTask(task_name, faceted_sc_projected_area)

    rHat_XB_B = []
    r_BN_N = []
    v_BN_N = []
    sigma_BN = []
    r_SN_N = []
    if general_heading:
        rHat_XB_B = np.array([1.0, 0.0, 0.0])
        body_heading_message_data = messaging.BodyHeadingMsgPayload()
        body_heading_message_data.rHat_XB_B = rHat_XB_B
        body_heading_message = messaging.BodyHeadingMsg().write(body_heading_message_data)
        faceted_sc_projected_area.bodyHeadingInMsg.subscribeTo(body_heading_message)

    elif sun_heading or velocity_heading:
        # Create the spacecraft state message
        r_BN_N = np.array([-4020338.690396649, 7490566.741852513, 5248299.211589362])  # [m]
        v_BN_N = np.array([-5199.77710904224, -3436.681645356935, 1041.576797498721])  # [m/s]
        theta_init = 10.0 * macros.D2R  # [rad]
        rot_axis_N = np.array([1.0, 0.0, 0.0])
        prv_BN = theta_init * rot_axis_N
        sigma_BN = rbk.PRV2MRP(prv_BN)
        spacecraft_state_message_data = messaging.SCStatesMsgPayload()
        spacecraft_state_message_data.r_BN_N = r_BN_N  # [m]
        spacecraft_state_message_data.v_BN_N = v_BN_N  # [m/s]
        spacecraft_state_message_data.sigma_BN = sigma_BN
        spacecraft_state_message = messaging.SCStatesMsg().write(spacecraft_state_message_data)
        faceted_sc_projected_area.spacecraftStateInMsg.subscribeTo(spacecraft_state_message)

        if sun_heading:
            # Create the Sun state ephemeris message
            r_SN_N = np.array([1.0, -10.0, 50.0])  # [m]
            sun_state_message_data = messaging.SpicePlanetStateMsgPayload()
            sun_state_message_data.PositionVector = r_SN_N  # [m]
            sun_state_message = messaging.SpicePlanetStateMsg().write(sun_state_message_data)
            faceted_sc_projected_area.sunStateInMsg.subscribeTo(sun_state_message)

    # Set up data logging
    total_projected_area_data_log = faceted_sc_projected_area.totalProjectedAreaOutMsg.recorder()
    facet_element_projected_area_data_log = []
    for outMsg in faceted_sc_projected_area.facetProjectedAreaOutMsgs:
        facet_element_projected_area_data_log.append(outMsg.recorder())
        test_sim.AddModelToTask(task_name, facet_element_projected_area_data_log[-1])
    test_sim.AddModelToTask(task_name, total_projected_area_data_log)

    # Execute simulation
    test_sim.InitializeSimulation()
    sim_time_1 = macros.sec2nano(2.0)  # [ns]
    test_sim.ConfigureStopTime(sim_time_1)
    test_sim.ExecuteSimulation()

    # Retrieve the logged data
    total_projected_area_sim = total_projected_area_data_log.area  # [m^2]
    facet_element_projected_area_list_sim = []
    for data in facet_element_projected_area_data_log:
        facet_element_projected_area_list_sim.append(data.area)

    # Compute truth data
    (facet_element_projected_area_list_truth,
     total_projected_area_truth) = compute_facet_projected_area(general_heading,
                                                                sun_heading,
                                                                velocity_heading,
                                                                num_facets,
                                                                rHat_XB_B,
                                                                r_BN_N,
                                                                v_BN_N,
                                                                sigma_BN,
                                                                r_SN_N,
                                                                facet_area_list,
                                                                facet_nHat_B_list)

    # Unit test check
    accuracy = 1e-12

    # Check per-facet projected area
    for idx in range(num_facets):
        np.testing.assert_allclose(
            facet_element_projected_area_list_sim[idx],
            facet_element_projected_area_list_truth[idx],
            atol=accuracy,
            verbose=True
        )

    # Check total projected area
    np.testing.assert_allclose(
        total_projected_area_sim,
        total_projected_area_truth,
        atol=accuracy,
        verbose=True
    )

def compute_facet_projected_area(general_heading,
                                 sun_heading,
                                 velocity_heading,
                                 num_facets,
                                 rHat_XB_B,
                                 r_BN_N,
                                 v_BN_N,
                                 sigma_BN,
                                 r_SN_N,
                                 facet_area_list,
                                 facet_nHat_B_list):
    # Compute the heading vector
    heading_hat_B = np.array([0.0, 0.0, 0.0])
    if general_heading:
        heading_hat_B = rHat_XB_B
    if sun_heading or velocity_heading:
        dcm_BN = rbk.MRP2C(sigma_BN)
        if velocity_heading:
            v_BN_B = dcm_BN @ v_BN_N  # [m/s]
            norm = np.linalg.norm(v_BN_B)  # [m/s]
            if norm > 1e-12:
                heading_hat_B = v_BN_B / norm
        else:
            r_SB_B = dcm_BN @ (r_SN_N - r_BN_N)  # [m]
            norm = np.linalg.norm(r_SB_B)  # [m]
            if norm > 1e-12:
                heading_hat_B = r_SB_B / norm

    # Compute the total and per-facet projected area
    projected_area_list_truth = []
    total_projected_area_truth = 0.0
    for idx in range(num_facets):
        cos_theta = np.dot(facet_nHat_B_list[idx], heading_hat_B)
        projected_area = facet_area_list[idx] * max(0.0, cos_theta)  # [m^2]

        projected_area_list_truth.append(projected_area)
        total_projected_area_truth += projected_area

    return projected_area_list_truth, total_projected_area_truth


if __name__=="__main__":
    test_facetedSpacecraftProjectedArea(
        True,  # show plots
        False,  # general_heading
        True,  # sun_heading
        False  # velocity_heading
    )
