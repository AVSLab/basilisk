
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
#   Module Name:        facetedSRPEffector
#   Author:             Leah Kiner
#

import numpy as np
import pytest
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import macros
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.simulation import facetedSpacecraftProjectedArea
from Basilisk.simulation import facetedSRPEffector
from Basilisk.simulation import spacecraft
from Basilisk.architecture import messaging

# Required constants
speed_light = 299792458.0  # [m/s] Speed of light
ast_u = 149597870700.0  # [m] Astronomical unit
solar_rad_flux = 1368.0  # [W/m^2] Solar radiation flux at 1 AU

@pytest.mark.parametrize(
    ("r_BN_N_init", "r_SN_N_init"),  # ([m], [m])
    [
        (np.array([-4020338.690396649, 7490566.741852513, 5248299.211589362]), np.array([0.0, 0.0, 0.0])),  # normal
        (np.array([7490566.741852513, 5248299.211589362, -4020338.690396649]), np.array([0.0, 0.0, 0.0])),  # normal
        (np.array([-4020338.690396649, 7490566.741852513, 5248299.211589362]), np.array([1.0e11, 2.0e10, -3.0e10])),  # nonzero Sun position
        (np.array([2.0e11, 0.0, 0.0]), np.array([0.0, 0.0, 0.0]))  # farther separation
    ]
)
@pytest.mark.parametrize("sigma_BN_init",  # [-]
    [
        np.array([0.0, 0.0, 0.0]),
        np.array([0.41421356, 0.0, 0.0]),  # +90 deg about x
        np.array([0.0, 0.41421356, 0.0]),  # +90 deg about y
        np.array([0.1, -0.2, 0.05]),  # arbitrary small rotation
    ]
)
@pytest.mark.parametrize("omega_BN_B_init",  # [rad/s]
    [
        np.array([0.0, 0.0, 0.0]),
        np.array([0.05, 0.0, 0.0]),
        np.array([0.0, 0.05, 0.0]),
        np.array([0.0, 0.0, 0.05]),
        np.array([0.05, -0.05, -0.05]),
    ]
)
def test_facetedSRPEffector(show_plots,
                            r_BN_N_init,
                            r_SN_N_init,
                            sigma_BN_init,
                            omega_BN_B_init):
    r"""
    **Verification Test Description**

    This unit test verifies that the faceted spacecraft solar radiation pressure (SRP) module correctly computes
    the aggregate force and torque acting on the spacecraft due to impinging photons from the Sun.
    The inertial Sun state information must be subscribed to the module ``SpicePlanetStateMsgPayload`` input message.
    The module assumes the spacecraft is modeled as a collection of facets, where the facet geometry information is
    passed as a vector of ``FacetElementBodyMsgPayload`` input messages to the module. The facet geometry information
    is required to be provided in the spacecraft body frame. This SRP module does not make any assumptions regarding
    whether the facets are rigid or articulate. The ``facetedSpacecraftModel`` module can be connected upstream and
    used to transform the facet geometry data from the facet frames to the spacecraft body frame. This upstream module
    can also be used to configure articulating facets. Finally, this SRP module also requires the sunlit area of all
    facets to be pre-computed and connected to the module vector of ``ProjectedAreaMsgPayload`` input messages.
    This information can be passed to the SRP module by connecting the facet geometry information to the
    ``facetedSpacecraftProjectedArea`` module upstream.

    This test sets up a simulation with a faceted spacecraft modeled as a cubic hub with two attached circular solar
    arrays. Six square facets represent the cubic hub and four circular facets represent the two solar arrays.
    The test varies the initial state information of both the spacecraft and the Sun. The test checks that the computed
    SRP forces and torques at each timestep match the values output from the module.

    **Test Parameters**

    Args:
        r_BN_N_init (np.ndarray): [m] shape (3,) Initial spacecraft inertial position
        r_SN_N_init (np.ndarray): [m] shape (3,) Initial Sun inertial position
        sigma_BN_init (np.ndarray): [-] shape (3,) Initial spacecraft inertial attitude
        omega_BN_B_init (np.ndarray): [rad/s] shape (3,) Initial spacecraft inertial angular velocity

    **Description of Variables Being Tested**

     The test checks that the module correctly computes the aggregate srp force and torque acting on the spacecraft
     body frame at each time step. The values logged from the module are checked to match the computed truth values.
    """

    task_name = "testTask"
    process_name = "testProcess"
    test_sim = SimulationBaseClass.SimBaseClass()
    test_time_step_sec = 0.001  # [s]
    test_process_rate = macros.sec2nano(test_time_step_sec)
    test_process = test_sim.CreateNewProcess(process_name)
    test_process.addTask(test_sim.CreateNewTask(task_name, test_process_rate))

    # Create the Sun spice inertial state message
    sun_state_message_data = messaging.SpicePlanetStateMsgPayload()
    sun_state_message_data.PositionVector = r_SN_N_init  # [m]
    sun_state_message_data.VelocityVector = [0.0, 0.0, 0.0]  # [m/s]
    sun_state_message = messaging.SpicePlanetStateMsg().write(sun_state_message_data)

    # Create the Sun
    grav_factory = simIncludeGravBody.gravBodyFactory()
    sun = grav_factory.createSun()
    sun.isCentralBody = True
    grav_factory.gravBodies['sun'].planetBodyInMsg.subscribeTo(sun_state_message)

    # Create the spacecraft hub
    sc_object = spacecraft.Spacecraft()
    sc_object.ModelTag = "scObject"
    sc_object.hub.mHub = 750.0  # [kg]
    sc_object.hub.r_BcB_B = [[0.0], [0.0], [1.0]]  # [m]
    sc_object.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]  # [kg m^2]
    sc_object.hub.r_CN_NInit = r_BN_N_init  # [m]
    sc_object.hub.v_CN_NInit = [[-5199.77710904224], [-3436.681645356935], [1041.576797498721]]  # [m/s]
    sc_object.hub.sigma_BNInit = sigma_BN_init  # [-]
    sc_object.hub.omega_BN_BInit = omega_BN_B_init  # [rad/s]

    # Facet geometry information
    num_facets = 10
    area_1 = 1.5 * 1.5  # [m^2]
    area_2 = np.pi * (0.5 * 7.5) * (0.5 * 7.5)  # [m^2]
    facet_area_list = [area_1, area_1, area_1, area_1, area_1, area_1, area_2, area_2, area_2, area_2]  # [m^2]
    facet_r_CopB_B_list = [np.array([0.75, 0.0, 0.0]),
                           np.array([0.0, 0.75, 0.0]),
                           np.array([-0.75, 0.0, 0.0]),
                           np.array([0.0, -0.75, 0.0]),
                           np.array([0.0, 0.0, 0.75]),
                           np.array([0.0, 0.0, -0.75]),
                           np.array([4.5, 0.0, 0.75]),
                           np.array([4.5, 0.0, 0.75]),
                           np.array([-4.5, 0.0, 0.75]),
                           np.array([-4.5, 0.0, 0.75])]  # [m]
    facet_nHat_B_list = [np.array([1.0, 0.0, 0.0]),
                          np.array([0.0, 1.0, 0.0]),
                          np.array([-1.0, 0.0, 0.0]),
                          np.array([0.0, -1.0, 0.0]),
                          np.array([0.0, 0.0, 1.0]),
                          np.array([0.0, 0.0, -1.0]),
                          np.array([0.0, 1.0, 0.0]),
                          np.array([0.0, -1.0, 0.0]),
                          np.array([0.0, 1.0, 0.0]),
                          np.array([0.0, -1.0, 0.0])]  # [-]
    facet_rotHat_B_list = [np.array([0.0, 0.0, 0.0]),
                           np.array([0.0, 0.0, 0.0]),
                           np.array([0.0, 0.0, 0.0]),
                           np.array([0.0, 0.0, 0.0]),
                           np.array([0.0, 0.0, 0.0]),
                           np.array([0.0, 0.0, 0.0]),
                           np.array([0.0, 0.0, 0.0]),
                           np.array([0.0, 0.0, 0.0]),
                           np.array([0.0, 0.0, 0.0]),
                           np.array([0.0, 0.0, 0.0])]  # [-]
    facet_diffuse_coeff_list = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
    facet_specular_coeff_list = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])

    # Create the facet element input messages
    facet_element_message_list = list()
    for idx in range(num_facets):
        facet_element_message_data = messaging.FacetElementBodyMsgPayload(
            area = facet_area_list[idx],
            r_CopB_B = facet_r_CopB_B_list[idx],
            nHat_B = facet_nHat_B_list[idx],
            rotHat_B = facet_rotHat_B_list[idx],
            c_diffuse = facet_diffuse_coeff_list[idx],
            c_specular = facet_specular_coeff_list[idx],
        )
        facet_element_message = messaging.FacetElementBodyMsg().write(facet_element_message_data)
        facet_element_message_list.append(facet_element_message)

    # Create the faceted spacecraft projected area module
    faceted_sc_projected_area = facetedSpacecraftProjectedArea.FacetedSpacecraftProjectedArea()
    faceted_sc_projected_area.ModelTag = "facetedSpacecraftProjectedArea"
    faceted_sc_projected_area.setNumFacets(num_facets)
    for idx in range(num_facets):
        faceted_sc_projected_area.facetElementBodyInMsgs[idx].subscribeTo(facet_element_message_list[idx])
    faceted_sc_projected_area.sunStateInMsg.subscribeTo(sun_state_message)
    faceted_sc_projected_area.spacecraftStateInMsg.subscribeTo(sc_object.scStateOutMsg)
    test_sim.AddModelToTask(task_name, faceted_sc_projected_area)
    test_sim.AddModelToTask(task_name, sc_object)

    # Create the faceted SRP effector module
    faceted_srp_effector = facetedSRPEffector.FacetedSRPEffector()
    faceted_srp_effector.ModelTag = "facetedSRPEffector"
    faceted_srp_effector.setNumFacets(num_facets)
    for idx in range(num_facets):
        faceted_srp_effector.facetElementBodyInMsgs[idx].subscribeTo(facet_element_message_list[idx])
        faceted_srp_effector.facetProjectedAreaInMsgs[idx].subscribeTo(faceted_sc_projected_area.facetProjectedAreaOutMsgs[idx])
    faceted_srp_effector.sunStateInMsg.subscribeTo(sun_state_message)
    sc_object.addDynamicEffector(faceted_srp_effector)
    test_sim.AddModelToTask(task_name, faceted_srp_effector)

    # Set up data logging
    spacecraft_state_data_log = sc_object.scStateOutMsg.recorder()
    sun_state_data_log = grav_factory.gravBodies['sun'].planetBodyInMsg.recorder()
    srp_data_log = faceted_srp_effector.logger(["forceExternal_B", "torqueExternalPntB_B"], test_process_rate)
    facet_element_projected_area_data_log = []
    for outMsg in faceted_sc_projected_area.facetProjectedAreaOutMsgs:
        facet_element_projected_area_data_log.append(outMsg.recorder())
        test_sim.AddModelToTask(task_name, facet_element_projected_area_data_log[-1])
    test_sim.AddModelToTask(task_name, spacecraft_state_data_log)
    test_sim.AddModelToTask(task_name, sun_state_data_log)
    test_sim.AddModelToTask(task_name, srp_data_log)

    # Execute the simulation
    test_sim.InitializeSimulation()
    sim_time = macros.sec2nano(0.01)  # [ns]
    test_sim.ConfigureStopTime(sim_time)
    test_sim.ExecuteSimulation()

    # Retrieve the logged data
    timespan = spacecraft_state_data_log.times() * macros.NANO2SEC  # [s]
    r_BN_N = spacecraft_state_data_log.r_BN_N  # [m]
    sigma_BN = spacecraft_state_data_log.sigma_BN  # [-]
    r_SN_N = sun_state_data_log.PositionVector  # [m]
    srp_force_B_list_sim = srp_data_log.forceExternal_B  # [N]
    srp_torque_B_list_sim = srp_data_log.torqueExternalPntB_B  # [Nm]
    facet_element_projected_area_list_sim = []
    for data in facet_element_projected_area_data_log:
        facet_element_projected_area_list_sim.append(data.area)

    # Compute truth data
    srp_force_B_list_truth, srp_torque_B_list_truth = compute_srp_force_torque(num_facets,
                                                                               facet_area_list,
                                                                               facet_r_CopB_B_list,
                                                                               facet_nHat_B_list,
                                                                               facet_diffuse_coeff_list,
                                                                               facet_specular_coeff_list,
                                                                               timespan,
                                                                               sigma_BN,
                                                                               r_BN_N,
                                                                               r_SN_N,
                                                                               facet_element_projected_area_list_sim)

    # Check the simulated srp force and torque values match the computed truth values
    for idx in range(len(timespan)):
        np.testing.assert_allclose(srp_force_B_list_sim[idx],
                                   srp_force_B_list_truth[idx],
                                   atol=1e-7,
                                   verbose=True)
        np.testing.assert_allclose(srp_torque_B_list_sim[idx],
                                   srp_torque_B_list_truth[idx],
                                   atol=1e-7,
                                   verbose=True)

def compute_srp_force_torque(num_facets,
                             facet_area_list,
                             facet_r_CopB_B_list,
                             facet_nHat_B_list,
                             facet_diffuse_coeff_list,
                             facet_specular_coeff_list,
                             timespan,
                             sigma_BN,
                             r_BN_N,
                             r_SN_N,
                             facet_element_projected_area_list_sim):

    srp_force_B_list_truth = []  # [N]
    srp_torque_B_list_truth = []  # [Nm]

    for time_idx in range(len(timespan)):

        # Determine unit direction vector pointing from sc to the Sun
        sigma_bn = sigma_BN[time_idx]  # [-]
        dcm_BN = rbk.MRP2C(sigma_bn)  # [-]
        r_bn_n = r_BN_N[time_idx]  # [m]
        r_BN_B = np.matmul(dcm_BN, r_bn_n)  # [m]
        r_sn_n = r_SN_N[time_idx]  # [m]
        r_SN_B = np.matmul(dcm_BN, r_sn_n)  # [m]
        r_SB_B = r_SN_B - r_BN_B  # [m]
        s_hat = r_SB_B / np.linalg.norm(r_SB_B)  # [-]

        # Determine the SRP pressure at the current sc location
        num_au = ast_u / np.linalg.norm(r_SB_B)
        srp_pressure = (solar_rad_flux / speed_light) * num_au * num_au  # [Pa]

        srp_force_B = np.zeros([3,])  # [N]
        srp_torque_B = np.zeros([3,])  # [Nm]
        for facet_idx in range(num_facets):

            projected_area = facet_element_projected_area_list_sim[facet_idx][time_idx]  # [m^2]

            if projected_area > 0:
                cos_theta = projected_area / facet_area_list[facet_idx]
                facet_force = -srp_pressure * projected_area * ((1 - facet_specular_coeff_list[facet_idx]) * s_hat
                                                                      + 2 * ( (facet_diffuse_coeff_list[facet_idx] / 3)
                                                                              + facet_specular_coeff_list[facet_idx] * cos_theta) * facet_nHat_B_list[facet_idx])  # [N]
                srp_force_B += facet_force  # [N]
                srp_torque_B += np.cross(facet_r_CopB_B_list[facet_idx], facet_force)  # [Nm]

        srp_force_B_list_truth.append(srp_force_B)
        srp_torque_B_list_truth.append(srp_torque_B)

    return np.array(srp_force_B_list_truth), np.array(srp_torque_B_list_truth)


if __name__=="__main__":
    test_facetedSRPEffector(
        False,  # show plots
        np.array([-4020338.690396649, 7490566.741852513, 5248299.211589362]),  # [m] r_BN_N_init
        np.array([0.0, 0.0, 0.0]),  # [m] r_SN_N_init
        np.array([0.0, 0.0, 0.0]),  # [-] sigma_BN_init
        np.array([0.0, 0.0, 0.0])  # [rad/s] omega_BN_B_init
    )
