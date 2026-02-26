
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
#   Module Name:        facetedSpacecraftModel
#   Author:             Leah Kiner
#   Creation Date:      Feb 10 2026
#   Last Updated:       Feb 10 2026
#

import numpy as np
import os
import pytest
from Basilisk import __path__

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.simulation import facetedSpacecraftModel
from Basilisk.architecture import messaging


@pytest.mark.parametrize("articulated_facet_1_initial_angle", [macros.D2R * 0.0, macros.D2R * 10.0, macros.D2R * 75.0, macros.D2R * -90.0])
@pytest.mark.parametrize("articulated_facet_1_intermediate_angle", [macros.D2R * -33.3, macros.D2R * 45.2, macros.D2R * 90.0, macros.D2R * 180.0])
@pytest.mark.parametrize("articulated_facet_1_final_angle", [macros.D2R * 0.0, macros.D2R * -5.4, macros.D2R * 22.5])
@pytest.mark.parametrize("fixed_facet_2_initial_angle", [macros.D2R * 0.0, macros.D2R * 10.0, macros.D2R * 75.0, macros.D2R * -90.0])
def test_facetedSpacecraftModel(show_plots,
                                articulated_facet_1_initial_angle,
                                articulated_facet_1_intermediate_angle,
                                articulated_facet_1_final_angle,
                                fixed_facet_2_initial_angle):
    r"""
    **Verification Test Description**

    This unit test verifies that the faceted spacecraft BSK module correctly transforms facet geometry data from
    the local facet frames to the spacecraft hub body frame. The module is configured to map either fixed or single-axis
    articulating facets to the spacecraft hub frame. The facet geometry input data to the BSK module is provided in
    each local facet frame. The facet geometry data is transformed and output from the BSK module in the spacecraft
    hub body frame. Articulating facets must be configured by calling the method
    addArticulatedFacet(Message<HingedRigidBodyMsgPayload> *tmpMsg) with a HingedRigidBodyMsgPayload message for each
    articulating facet. The module assumes the articulating facets are added first to the module, followed by any
    fixed facets.

    This test sets up the faceted spacecraft BSK module with two facets. The first facet articulates while the second
    is fixed. A stand-alone HingedRigidBodyMsgPayload is provided for the articulated facet. To ensure the module
    correctly handles articulating facets, the simulation is broken up into two segments. The first segment provides
    an intermediate articulation angle while the second segment provides a final articulation angle. The test varies
    the articulated facet initial angle, intermediate angle, and final angle. The initial angle for the initial facet
    is also varied in the test.

    **Test Parameters**

    Args:
        articulated_facet_1_initial_angle (float): [rad] Initial articulated facet 1 angle
        articulated_facet_1_intermediate_angle (float): [rad] Intermediate articulated facet 1 angle
        articulated_facet_1_final_angle (float): [steps] Final articulated facet 1 angle
        fixed_facet_2_initial_angle (float): [rad] Initial fixed facet 2 angle

    **Description of Variables Being Tested**

     The test checks that the facet geometry data is correctly transformed to the spacecraft hub body frame. The
     specific variables checked are the facet center of pressure locations ``r_CopB_B``, the facet normal
     vectors ``nHat_B``, and the facet articulation axes ``rotHat_B``.
    """

    task_name = "unitTask"
    process_name = "TestProcess"
    test_sim = SimulationBaseClass.SimBaseClass()
    test_time_step_sec = 1.0
    test_process_rate = macros.sec2nano(test_time_step_sec)
    test_process = test_sim.CreateNewProcess(process_name)
    test_process.addTask(test_sim.CreateNewTask(task_name, test_process_rate))

    # Facet geometry information
    num_facets = 2
    facet_area_list = [0.5, 1.0]
    facet_r_CopF_F_list = [np.array([-0.1, 0.1, -0.1]),
                           np.array([0.1, -0.1, -0.1])]
    facet_nHat_F_list = [np.array([1.0, 0.0, 0.0]),
                         np.array([0.0, 1.0, 0.0])]
    facet_rotHat_F_list = [np.array([0.0, 1.0, 0.0]),
                           np.array([0.0, 0.0, 1.0])]
    facet_r_FB_B_list = [np.array([0.0, 1.0, 0.0]),
                         np.array([0.0, 0.0, -1.0])]
    facet_diffuse_coeff_list = [0.1, 0.1]
    facet_specular_coeff_list = [0.9, 0.9]
    prv_F01B = articulated_facet_1_initial_angle * facet_rotHat_F_list[0]
    prv_F02B = fixed_facet_2_initial_angle * facet_rotHat_F_list[1]
    facet_dcm_F0B_list = [rbk.PRV2C(prv_F01B),
                          rbk.PRV2C(prv_F02B)]

    # Create the facet element input messages
    articulated_facet_1_element_message_data = messaging.FacetElementMsgPayload(
        area = facet_area_list[0],
        r_CopF_F = facet_r_CopF_F_list[0],
        nHat_F = facet_nHat_F_list[0],
        rotHat_F = facet_rotHat_F_list[0],
        dcm_F0B = facet_dcm_F0B_list[0],
        r_FB_B = facet_r_FB_B_list[0],
        c_diffuse = facet_diffuse_coeff_list[0],
        c_specular = facet_specular_coeff_list[0],
    )
    fixed_facet_2_element_message_data = messaging.FacetElementMsgPayload(
        area = facet_area_list[1],
        r_CopF_F = facet_r_CopF_F_list[1],
        nHat_F = facet_nHat_F_list[1],
        rotHat_F = facet_rotHat_F_list[1],
        dcm_F0B = facet_dcm_F0B_list[1],
        r_FB_B = facet_r_FB_B_list[1],
        c_diffuse = facet_diffuse_coeff_list[1],
        c_specular = facet_specular_coeff_list[1],
    )
    articulated_facet_1_element_message = messaging.FacetElementMsg().write(articulated_facet_1_element_message_data)
    fixed_facet_2_element_message = messaging.FacetElementMsg().write(fixed_facet_2_element_message_data)

    # Create the articulated facet angle message (facet 1)
    articulated_facet_angle_message_data = messaging.HingedRigidBodyMsgPayload()
    articulated_facet_angle_message_data.theta = articulated_facet_1_intermediate_angle  # [rad]
    articulated_facet_angle_message_data.thetaDot = 0.0  # [rad]
    articulated_facet_angle_message = messaging.HingedRigidBodyMsg().write(articulated_facet_angle_message_data)

    # Create the faceted spacecraft module
    faceted_sc_model = facetedSpacecraftModel.FacetedSpacecraftModel()
    faceted_sc_model.ModelTag = "facetedSCModel"
    faceted_sc_model.setNumTotalFacets(num_facets)
    faceted_sc_model.addArticulatedFacet(articulated_facet_angle_message)
    faceted_sc_model.facetElementInMsgs[0].subscribeTo(articulated_facet_1_element_message)
    faceted_sc_model.facetElementInMsgs[1].subscribeTo(fixed_facet_2_element_message)
    test_sim.AddModelToTask(task_name, faceted_sc_model)

    # Set up data logging
    facet_element_body_data_log = []
    for outMsg in faceted_sc_model.facetElementBodyOutMsgs:
        facet_element_body_data_log.append(outMsg.recorder())
        test_sim.AddModelToTask(task_name, facet_element_body_data_log[-1])

    # Execute simulation chunk 1
    test_sim.InitializeSimulation()
    sim_time_1 = macros.sec2nano(2.0)
    test_sim.ConfigureStopTime(sim_time_1)
    test_sim.ExecuteSimulation()

    # Create the second articulated facet angle message (facet 1)
    articulated_facet_angle_message_data = messaging.HingedRigidBodyMsgPayload()
    articulated_facet_angle_message_data.theta = articulated_facet_1_final_angle  # [rad]
    articulated_facet_angle_message_data.thetaDot = 0.0  # [rad]
    articulated_facet_angle_message = messaging.HingedRigidBodyMsg().write(articulated_facet_angle_message_data)
    faceted_sc_model.articulatedFacetDataInMsgs[0].subscribeTo(articulated_facet_angle_message)

    # Execute simulation chunk 2
    sim_time_2 = macros.sec2nano(3.0)
    test_sim.ConfigureStopTime(sim_time_1 + sim_time_2)
    test_sim.ExecuteSimulation()

    # Retrieve the logged data
    facet_r_CopB_B_list_sim = []
    facet_nHat_B_list_sim = []
    facet_rotHat_B_list_sim = []
    for data in facet_element_body_data_log:
        facet_r_CopB_B_list_sim.append(data.r_CopB_B)
        facet_nHat_B_list_sim.append(data.nHat_B)
        facet_rotHat_B_list_sim.append(data.rotHat_B)

    # Compute truth data
    (facet_r_CopB_B_list_truth,
    facet_nHat_B_list_truth,
    facet_rotHat_B_list_truth) = compute_facet_body_data(facet_r_CopF_F_list,
                                                         facet_nHat_F_list,
                                                         facet_rotHat_F_list,
                                                         facet_r_FB_B_list,
                                                         facet_dcm_F0B_list,
                                                         articulated_facet_1_intermediate_angle,
                                                         articulated_facet_1_final_angle)

    # Unit test check
    accuracy = 1e-12

    # Check facet 1 data
    articulated_facet_1_data_to_check = [
        (facet_r_CopB_B_list_sim[0][0:3], facet_r_CopB_B_list_truth[0]),
        (facet_nHat_B_list_sim[0][0:3], facet_nHat_B_list_truth[0]),
        (facet_rotHat_B_list_sim[0][0:3], facet_rotHat_B_list_truth[0]),
        (facet_r_CopB_B_list_sim[0][3:6], facet_r_CopB_B_list_truth[1]),
        (facet_r_CopB_B_list_sim[0][3:6], facet_r_CopB_B_list_truth[1]),
        (facet_r_CopB_B_list_sim[0][3:6], facet_r_CopB_B_list_truth[1])
    ]
    for sim_data, truth_data in articulated_facet_1_data_to_check:
        for row in sim_data:
            np.testing.assert_allclose(
                row,
                truth_data,
                atol=accuracy,
                verbose=True
            )

    # Check facet 2 data
    fixed_facet_2_data_to_check = [
        (facet_r_CopB_B_list_sim[1], facet_r_CopB_B_list_truth[2]),
        (facet_nHat_B_list_sim[1], facet_nHat_B_list_truth[2]),
        (facet_rotHat_B_list_sim[1], facet_rotHat_B_list_truth[2]),
    ]
    for sim_data, truth_data in fixed_facet_2_data_to_check:
        for row in sim_data:
            np.testing.assert_allclose(
                row,
                truth_data,
                atol=accuracy,
                verbose=True
            )

def compute_facet_body_data(facet_r_CopF_F_list,
                            facet_nHat_F_list,
                            facet_rotHat_F_list,
                            facet_r_FB_B_list,
                            facet_dcm_F0B_list,
                            articulated_facet_1_intermediate_angle,
                            articulated_facet_1_final_angle):

    # Compute articulated facet 1 truth data for simulation segment 1
    prv_FF0 = articulated_facet_1_intermediate_angle * facet_rotHat_F_list[0]
    dcm_FF0 = rbk.PRV2C(prv_FF0)
    dcm_FB = np.matmul(dcm_FF0, facet_dcm_F0B_list[0])
    articulated_facet_1_r_CopB_B_intermediate = np.matmul(dcm_FB.transpose(), facet_r_CopF_F_list[0]) + facet_r_FB_B_list[0]
    articulated_facet_1_nHat_B_intermediate = np.matmul(dcm_FB.transpose(), facet_nHat_F_list[0])
    articulated_facet_1_rotHat_B_intermediate = np.matmul(dcm_FB.transpose(), facet_rotHat_F_list[0])

    # Compute articulated facet 1 truth data for simulation segment 2
    prv_FF0 = articulated_facet_1_final_angle * facet_rotHat_F_list[0]
    dcm_FF0 = rbk.PRV2C(prv_FF0)
    dcm_FB = np.matmul(dcm_FF0, facet_dcm_F0B_list[0])
    articulated_facet_1_r_CopB_B_final = np.matmul(dcm_FB.transpose(), facet_r_CopF_F_list[0]) + facet_r_FB_B_list[0]
    articulated_facet_1_nHat_B_final = np.matmul(dcm_FB.transpose(), facet_nHat_F_list[0])
    articulated_facet_1_rotHat_B_final = np.matmul(dcm_FB.transpose(), facet_rotHat_F_list[0])

    # Compute fixed facet 2 truth data
    fixed_facet_2_r_CopB_B = np.matmul(facet_dcm_F0B_list[1].transpose(), facet_r_CopF_F_list[1]) + facet_r_FB_B_list[1]
    fixed_facet_2_nHat_B = np.matmul(facet_dcm_F0B_list[1].transpose(), facet_nHat_F_list[1])
    fixed_facet_2_rotHat_B = np.matmul(facet_dcm_F0B_list[1].transpose(), facet_rotHat_F_list[1])

    # Format and output truth data as lists
    facet_r_CopB_B_list_truth = [articulated_facet_1_r_CopB_B_intermediate,
                                 articulated_facet_1_r_CopB_B_final,
                                 fixed_facet_2_r_CopB_B]
    facet_nHat_B_list_truth = [articulated_facet_1_nHat_B_intermediate,
                               articulated_facet_1_nHat_B_final,
                               fixed_facet_2_nHat_B]
    facet_rotHat_B_list_truth = [articulated_facet_1_rotHat_B_intermediate,
                                 articulated_facet_1_rotHat_B_final,
                                 fixed_facet_2_rotHat_B]

    return facet_r_CopB_B_list_truth, facet_nHat_B_list_truth, facet_rotHat_B_list_truth

if __name__=="__main__":
    test_facetedSpacecraftModel(
        True,  # show plots
        10.0 * macros.D2R,  # [rad] Articulated facet 1 initial angle
        45.0 * macros.D2R,  # [rad] Articulated facet 1 intermediate angle
        110.0 * macros.D2R,  # [rad] Articulated facet 1 final angle
        -30.0 * macros.D2R,  # [rad] Fixed facet 2 initial angle
    )
