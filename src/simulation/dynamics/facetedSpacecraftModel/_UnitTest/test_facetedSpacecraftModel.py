
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
@pytest.mark.parametrize("articulated_facet_2_initial_angle", [macros.D2R * 0.0, macros.D2R * -10.0, macros.D2R * 90.0, macros.D2R * 180.0])
@pytest.mark.parametrize("articulated_facet_1_intermediate_angle", [macros.D2R * -33.3, macros.D2R * 45.2, macros.D2R * 90.0, macros.D2R * 180.0])
@pytest.mark.parametrize("articulated_facet_2_intermediate_angle", [macros.D2R * -28.0, macros.D2R * 45.2, macros.D2R * -90.0, macros.D2R * 180.0])
@pytest.mark.parametrize("articulated_facet_1_final_angle", [macros.D2R * 0.0, macros.D2R * -5.4, macros.D2R * 22.5])
@pytest.mark.parametrize("articulated_facet_2_final_angle", [macros.D2R * 0.0, macros.D2R * -14.0, macros.D2R * 22.5])
@pytest.mark.parametrize("fixed_facet_3_angle", [macros.D2R * 0.0, macros.D2R * 10.0, macros.D2R * 75.0, macros.D2R * -90.0])
@pytest.mark.parametrize("fixed_facet_4_angle", [macros.D2R * 0.0, macros.D2R * -10.0, macros.D2R * 90.0, macros.D2R * 180.0])

def test_facetedSpacecraftModel(show_plots,
                                articulated_facet_1_initial_angle,
                                articulated_facet_2_initial_angle,
                                articulated_facet_1_intermediate_angle,
                                articulated_facet_2_intermediate_angle,
                                articulated_facet_1_final_angle,
                                articulated_facet_2_final_angle,
                                fixed_facet_3_angle,
                                fixed_facet_4_angle):

    task_name = "unitTask"
    process_name = "TestProcess"
    test_sim = SimulationBaseClass.SimBaseClass()
    test_time_step_sec = 1.0
    test_process_rate = macros.sec2nano(test_time_step_sec)
    test_process = test_sim.CreateNewProcess(process_name)
    test_process.addTask(test_sim.CreateNewTask(task_name, test_process_rate))

    # Facet information
    num_facets = 4
    num_articulated_facets = 2
    facet_area_list = [0.5, 0.75, 1.0, 1.25]
    facet_r_CopF_F_list = [np.array([0.0, 0.1, 0.0]),
                           np.array([0.0, 0.0, 0.0]),
                           np.array([0.1, 0.0, -0.1]),
                           np.array([0.0, 0.0, 0.1])]
    facet_nHat_F_list = [np.array([1.0, 0.0, 0.0]),
                         np.array([0.0, -1.0, 0.0]),
                         np.array([0.0, 1.0, 0.0]),
                         np.array([1.0, 0.0, 0.0])]
    facet_rotHat_F_list = [np.array([1.0, 0.0, 0.0]),
                           np.array([0.0, 1.0, 0.0]),
                           np.array([0.0, 0.0, 1.0]),
                           np.array([0.0, 0.0, -1.0])]
    facet_r_FB_B_list = [np.array([0.0, 0.0, 0.0]),
                         np.array([1.0, 0.0, 0.0]),
                         np.array([0.0, 1.0, 0.0]),
                         np.array([0.0, 0.0, -1.0])]
    facet_diffuse_coeff_list = [0.1, 0.1, 0.1, 0.1]
    facet_specular_coeff_list = [0.9, 0.9, 0.9, 0.9]
    prv_F01B = articulated_facet_1_initial_angle * facet_rotHat_F_list[0]
    prv_F02B = articulated_facet_2_initial_angle * facet_rotHat_F_list[1]
    prv_F03B = fixed_facet_3_angle * facet_rotHat_F_list[2]
    prv_F04B = fixed_facet_4_angle * facet_rotHat_F_list[3]
    facet_dcm_F0B_list = [rbk.PRV2C(prv_F01B),
                          rbk.PRV2C(prv_F02B),
                          rbk.PRV2C(prv_F03B),
                          rbk.PRV2C(prv_F04B)]

    # Create the faceted spacecraft message
    faceted_sc_message_data = messaging.FacetedSCMsgPayload(
        numFacets = num_facets,
        numArticulatedFacets = num_articulated_facets,
        facets = [
            messaging.FacetElementMsgPayload(
                area = facet_area_list[idx],
                r_CopF_F = facet_r_CopF_F_list[idx],
                nHat_F = facet_nHat_F_list[idx],
                rotHat_F = facet_rotHat_F_list[idx],
                dcm_F0B = facet_dcm_F0B_list[idx],
                r_FB_B = facet_r_FB_B_list[idx],
                c_diffuse = facet_diffuse_coeff_list[idx],
                c_specular = facet_specular_coeff_list[idx],
            )
            for idx in range(num_facets)
        ]
    )
    faceted_sc_message = messaging.FacetedSCMsg().write(faceted_sc_message_data)









    # Create the first articulated facet angle messages
    articulated_facet_1_angle_message_data = messaging.HingedRigidBodyMsgPayload()
    articulated_facet_1_angle_message_data.theta = articulated_facet_1_intermediate_angle  # [rad]
    articulated_facet_1_angle_message_data.thetaDot = 0.0  # [rad]
    articulated_facet_1_angle_message = messaging.HingedRigidBodyMsg().write(articulated_facet_1_angle_message_data)

    articulated_facet_2_angle_message_data = messaging.HingedRigidBodyMsgPayload()
    articulated_facet_2_angle_message_data.theta = articulated_facet_2_intermediate_angle  # [rad]
    articulated_facet_2_angle_message_data.thetaDot = 0.0  # [rad]
    articulated_facet_2_angle_message = messaging.HingedRigidBodyMsg().write(articulated_facet_2_angle_message_data)

    # Create the faceted spacecraft module
    faceted_sc_model = facetedSpacecraftModel.FacetedSpacecraftModel()
    faceted_sc_model.ModelTag = "facetedSCModel"
    faceted_sc_model.addArticulatedFacet(articulated_facet_1_angle_message)
    faceted_sc_model.addArticulatedFacet(articulated_facet_2_angle_message)
    faceted_sc_model.articulatedFacetDataInMsgs[0].subscribeTo(articulated_facet_1_angle_message)
    faceted_sc_model.articulatedFacetDataInMsgs[1].subscribeTo(articulated_facet_2_angle_message)
    faceted_sc_model.facetedSCInMsg.subscribeTo(faceted_sc_message)
    test_sim.AddModelToTask(task_name, faceted_sc_model)

    # Set up data logging
    faceted_sc_body_data_log = faceted_sc_model.facetedSCBodyOutMsg.recorder()
    test_sim.AddModelToTask(task_name, faceted_sc_body_data_log)

    # Execute simulation chunk 1
    test_sim.InitializeSimulation()
    sim_time_1 = macros.sec2nano(3.0)
    test_sim.ConfigureStopTime(sim_time_1)
    test_sim.ExecuteSimulation()

    # Create the second articulated facet angle messages
    articulated_facet_1_angle_message_data = messaging.HingedRigidBodyMsgPayload()
    articulated_facet_1_angle_message_data.theta = articulated_facet_1_final_angle  # [rad]
    articulated_facet_1_angle_message_data.thetaDot = 0.0  # [rad]
    articulated_facet_1_angle_message = messaging.HingedRigidBodyMsg().write(articulated_facet_1_angle_message_data)
    faceted_sc_model.articulatedFacetDataInMsgs[0].subscribeTo(articulated_facet_1_angle_message)

    articulated_facet_2_angle_message_data = messaging.HingedRigidBodyMsgPayload()
    articulated_facet_2_angle_message_data.theta = articulated_facet_2_final_angle  # [rad]
    articulated_facet_2_angle_message_data.thetaDot = 0.0  # [rad]
    articulated_facet_2_angle_message = messaging.HingedRigidBodyMsg().write(articulated_facet_2_angle_message_data)
    faceted_sc_model.articulatedFacetDataInMsgs[1].subscribeTo(articulated_facet_2_angle_message)

    # Execute simulation chunk 2
    sim_time_2 = macros.sec2nano(3.0)
    test_sim.ConfigureStopTime(sim_time_1 + sim_time_2)
    test_sim.ExecuteSimulation()

    # Retrieve the logged data
    timespan = faceted_sc_body_data_log.times() * macros.NANO2SEC  # [s]
    facet_1_body_data_list = faceted_sc_body_data_log.facets[0]
    facet_2_body_data_list = faceted_sc_body_data_log.facets[1]
    facet_3_body_data_list = faceted_sc_body_data_log.facets[2]
    facet_4_body_data_list = faceted_sc_body_data_log.facets[3]

    facet_r_CopB_B_list_sim = [facet_1_body_data_list.r_CopB_B,
                               facet_2_body_data_list.r_CopB_B,
                               facet_3_body_data_list.r_CopB_B,
                               facet_4_body_data_list.r_CopB_B]
    facet_nHat_B_list_sim = [facet_1_body_data_list.nHat_B,
                             facet_2_body_data_list.nHat_B,
                             facet_3_body_data_list.nHat_B,
                             facet_4_body_data_list.nHat_B]
    facet_rotHat_B_list_sim =[facet_1_body_data_list.rotHat_B,
                              facet_2_body_data_list.rotHat_B,
                              facet_3_body_data_list.rotHat_B,
                              facet_4_body_data_list.rotHat_B]

    # Compute truth data
    (facet_r_CopB_B_list_truth,
    facet_nHat_B_list_truth,
    facet_rotHat_B_list_truth) = compute_facet_body_data(facet_r_CopF_F_list,
                                                         facet_nHat_F_list,
                                                         facet_rotHat_F_list,
                                                         facet_r_FB_B_list,
                                                         facet_dcm_F0B_list,
                                                         articulated_facet_1_intermediate_angle,
                                                         articulated_facet_2_intermediate_angle,
                                                         articulated_facet_1_final_angle,
                                                         articulated_facet_2_final_angle)

    print("\n\nfacet_r_CopB_B_list_truth")
    print(facet_r_CopB_B_list_truth)
    print("facet_r_CopB_B_list_sim")
    print(facet_r_CopB_B_list_sim)

    print("\n\nfacet_nHat_B_list_truth")
    print(facet_nHat_B_list_truth)
    print("facet_nHat_B_list_sim")
    print(facet_nHat_B_list_sim)

    print("\n\nfacet_rotHat_B_list_truth")
    print(facet_rotHat_B_list_truth)
    print("facet_rotHat_B_list_sim")
    print(facet_rotHat_B_list_sim)

    # # Check the simulation body vectors match the truth values
    # accuracy = 1e-12
    # np.testing.assert_allclose(facet_r_CopB_B_list_sim,
    #                            facet_r_CopB_B_list_truth,
    #                            atol=accuracy,
    #                            verbose=True)
    #
    # np.testing.assert_allclose(facet_nHat_B_list_sim,
    #                            facet_nHat_B_list_truth,
    #                            atol=accuracy,
    #                            verbose=True)
    #
    # np.testing.assert_allclose(facet_rotHat_B_list_sim,
    #                            facet_rotHat_B_list_truth,
    #                            atol=accuracy,
    #                            verbose=True)

def compute_facet_body_data(facet_r_CopF_F_list,
                            facet_nHat_F_list,
                            facet_rotHat_F_list,
                            facet_r_FB_B_list,
                            facet_dcm_F0B_list,
                            articulated_facet_1_intermediate_angle,
                            articulated_facet_2_intermediate_angle,
                            articulated_facet_1_final_angle,
                            articulated_facet_2_final_angle):

    facet_3_r_CopB_B = np.matmul(facet_dcm_F0B_list[2].transpose(), facet_r_CopF_F_list[2]) + facet_r_FB_B_list[2]
    facet_3_nHat_B = np.matmul(facet_dcm_F0B_list[2].transpose(), facet_nHat_F_list[2])
    facet_3_rotHat_B = np.matmul(facet_dcm_F0B_list[2].transpose(), facet_rotHat_F_list[2])

    facet_4_r_CopB_B = np.matmul(facet_dcm_F0B_list[3].transpose(), facet_r_CopF_F_list[3]) + facet_r_FB_B_list[3]
    facet_4_nHat_B = np.matmul(facet_dcm_F0B_list[3].transpose(), facet_nHat_F_list[3])
    facet_4_rotHat_B = np.matmul(facet_dcm_F0B_list[3].transpose(), facet_rotHat_F_list[3])

    facet_r_CopB_B_list_truth = []
    facet_nHat_B_list_truth = []
    facet_rotHat_B_list_truth = []
    for idx_segment in range(3):
        for idx_facet in range(4):
            if idx_facet == 0:
                if idx_segment == 0:
                    dcm_FB = facet_dcm_F0B_list[0]
                elif idx_segment == 1:
                    angle = articulated_facet_1_intermediate_angle
                    prv_FF0 = angle * facet_rotHat_F_list[idx_facet]
                    dcm_FF0 = rbk.PRV2C(prv_FF0)
                    dcm_FB = np.matmul(dcm_FF0, facet_dcm_F0B_list[idx_facet])
                else:
                    angle = articulated_facet_1_final_angle
                    prv_FF0 = angle * facet_rotHat_F_list[idx_facet]
                    dcm_FF0 = rbk.PRV2C(prv_FF0)
                    dcm_FB = np.matmul(dcm_FF0, facet_dcm_F0B_list[idx_facet])

                facet_r_CopB_B = np.matmul(dcm_FB.transpose(), facet_r_CopF_F_list[idx_facet]) + facet_r_FB_B_list[idx_facet]
                facet_nHat_B = np.matmul(dcm_FB.transpose(), facet_nHat_F_list[idx_facet])
                facet_rotHat_B = np.matmul(dcm_FB.transpose(), facet_rotHat_F_list[idx_facet])

                facet_r_CopB_B_list_truth.append(facet_r_CopB_B)
                facet_nHat_B_list_truth.append(facet_nHat_B)
                facet_rotHat_B_list_truth.append(facet_rotHat_B)
            elif idx_facet == 1:
                if idx_segment == 0:
                    dcm_FB = facet_dcm_F0B_list[1]
                elif idx_segment == 1:
                    angle = articulated_facet_2_intermediate_angle
                    prv_FF0 = angle * facet_rotHat_F_list[idx_facet]
                    dcm_FF0 = rbk.PRV2C(prv_FF0)
                    dcm_FB = np.matmul(dcm_FF0, facet_dcm_F0B_list[idx_facet])
                else:
                    angle = articulated_facet_2_final_angle
                    prv_FF0 = angle * facet_rotHat_F_list[idx_facet]
                    dcm_FF0 = rbk.PRV2C(prv_FF0)
                    dcm_FB = np.matmul(dcm_FF0, facet_dcm_F0B_list[idx_facet])

                facet_r_CopB_B = np.matmul(dcm_FB.transpose(), facet_r_CopF_F_list[idx_facet]) + facet_r_FB_B_list[idx_facet]
                facet_nHat_B = np.matmul(dcm_FB.transpose(), facet_nHat_F_list[idx_facet])
                facet_rotHat_B = np.matmul(dcm_FB.transpose(), facet_rotHat_F_list[idx_facet])

                facet_r_CopB_B_list_truth.append(facet_r_CopB_B)
                facet_nHat_B_list_truth.append(facet_nHat_B)
                facet_rotHat_B_list_truth.append(facet_rotHat_B)
            elif idx_facet == 2:
                facet_r_CopB_B_list_truth.append(facet_3_r_CopB_B)
                facet_nHat_B_list_truth.append(facet_3_nHat_B)
                facet_rotHat_B_list_truth.append(facet_3_rotHat_B)
            else:
                facet_r_CopB_B_list_truth.append(facet_4_r_CopB_B)
                facet_nHat_B_list_truth.append(facet_4_nHat_B)
                facet_rotHat_B_list_truth.append(facet_4_rotHat_B)

    return facet_r_CopB_B_list_truth, facet_nHat_B_list_truth, facet_rotHat_B_list_truth

if __name__=="__main__":
    test_facetedSpacecraftModel(
        True,  # show plots
        0.0 * macros.D2R,  # [rad] Articulated facet 1 initial angle
        0.0 * macros.D2R,  # [rad] Articulated facet 2 initial angle
        0.0 * macros.D2R,  # [rad] Articulated facet 1 intermediate angle
        0.0 * macros.D2R,  # [rad] Articulated facet 2 intermediate angle
        0.0 * macros.D2R,  # [rad] Articulated facet 2 final angle
        0.0 * macros.D2R,  # [rad] Articulated facet 2 final angle
        0.0 * macros.D2R,  # [rad] Fixed facet 3 angle
        0.0 * macros.D2R,  # [rad] Fixed facet 4 angle
    )
