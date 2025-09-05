import math
import os

import matplotlib.pyplot as plt
import numpy as np

from Basilisk import __path__
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import alMeanOEFeedback
from Basilisk.fswAlgorithms import mrpFeedback
from Basilisk.fswAlgorithms import attTrackingError
from Basilisk.fswAlgorithms import hillPoint
from Basilisk.fswAlgorithms import inertial3D
from Basilisk.fswAlgorithms import thrFiringSchmitt
from Basilisk.fswAlgorithms import thrForceMapping
from Basilisk.simulation import extForceTorque
from Basilisk.simulation import simpleNav
from Basilisk.simulation import GravityGradientEffector
from Basilisk.simulation import spacecraft
from Basilisk.simulation import thrusterDynamicEffector, fuelTank
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import RigidBodyKinematics
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import unitTestSupport
from Basilisk.utilities import fswSetupThrusters
from Basilisk.utilities import simIncludeThruster

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


def run(numOrbits):

    # mission parameters
    sat_mass = 56  # [kg]
    I = [4.42, 0.0, 0.0, 0.0, 5.59, 0.0, 0.0, 0.0, 9.69]

    degree_window_apses = 11  # [deg], half-width of the thrust windows
    degree_window_nodes = 15  # [deg], half-width of the thrust windows

    # formation parameters
    altitude = 600  # [km]
    phase_angle = 45  # [rad]
    ring_diameter = 10 * 1e3  # [m]
    ring_height = ring_diameter / 2  # [m]

    # chief and deputy orbital elements at steady state
    chief_a = 6975286.459  # [m]
    chief_e = 0.00699039
    chief_i = np.deg2rad(98.157583)  # [deg]
    chief_Omega = np.deg2rad(10.301249)  # [deg]
    chief_omega = np.deg2rad(324.769912)  # [deg]
    chief_f = np.deg2rad(12.527474)  # [deg]

    deputy_a = 6975269.894  # [m]
    deputy_e = 0.00740837
    deputy_i = np.deg2rad(98.174005)  # [deg]
    deputy_Omega = np.deg2rad(10.307787)  # [deg]
    deputy_omega = np.deg2rad(329.392277)  # [deg]
    deputy_f = np.deg2rad(7.840825)  # [deg]

    # thruster parameters
    max_thrust = 0.0022  # [N]
    Isp_sec = 2500  #  NPT30 [s]
    m_prop = 1.0  #  [kg]
    max_thruster_lifetime = 2520  #  [hr] +/= 210 hours for the cathode degradations over time (both filaments) estimated every 100 firings is < 4% of the total lifetime

    thruster_location = [
        [0.48417, -0.13291, 0],
        [0.48417, 0.13291, 0],
    ]  # thruster location from Mike's google chat message 5/27/2025

    thrust_direction = [
        [-1.0, 0.0, 0.0],
        [-1.0, 0.0, 0.0],
    ]  # body-frame

    # create simulation
    sc_sim = SimulationBaseClass.SimBaseClass()
    sc_sim.SetProgressBar(True)

    # dynamics process and task
    dynProcessName = "dynProcess"
    dyn_task_name = "dynTask"
    dynProcess = sc_sim.CreateNewProcess(dynProcessName, 2)
    dyn_time_step = macros.sec2nano(15.0)
    dynProcess.addTask(sc_sim.CreateNewTask(dyn_task_name, dyn_time_step))

    # creating spacecraft objects
    chief_obj = spacecraft.Spacecraft()
    chief_obj.ModelTag = "chief_obj"
    chief_obj.hub.mHub = sat_mass  # [kg]
    chief_obj.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    chief_obj.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)
    sc_sim.AddModelToTask(dyn_task_name, chief_obj, 2)

    deputy_obj = spacecraft.Spacecraft()
    deputy_obj.ModelTag = "deputy_obj"
    deputy_obj.hub.mHub = sat_mass
    deputy_obj.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    deputy_obj.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)
    sc_sim.AddModelToTask(dyn_task_name, deputy_obj, 2)

    # creating gravity
    grav_factor = simIncludeGravBody.gravBodyFactory()
    earth = grav_factor.createEarth()
    earth.isCentralBody = True
    mu = earth.mu
    earth.useSphericalHarmonicsGravityModel(
        bskPath + "/supportData/LocalGravData/GGM03S.txt", 2
    )
    grav_factor.addBodiesTo(chief_obj)
    grav_factor.addBodiesTo(deputy_obj)

    # gravity gradient
    chief_gg = GravityGradientEffector.GravityGradientEffector()
    chief_gg.ModelTag = chief_obj.ModelTag
    chief_gg.addPlanetName(earth.planetName)
    chief_obj.addDynamicEffector(chief_gg)
    sc_sim.AddModelToTask(dyn_task_name, chief_gg)

    deputy_gg = GravityGradientEffector.GravityGradientEffector()
    deputy_gg.ModelTag = deputy_obj.ModelTag
    deputy_gg.addPlanetName(earth.planetName)
    deputy_obj.addDynamicEffector(deputy_gg)
    sc_sim.AddModelToTask(dyn_task_name, deputy_gg)

    # configure dynamics of the electric thruster
    thruster_dynamics = thrusterDynamicEffector.ThrusterDynamicEffector()
    sc_sim.AddModelToTask(dyn_task_name, thruster_dynamics)

    # Make a fresh thruster factory instance, this is critical to run multiple times
    thruster_factory = simIncludeThruster.thrusterFactory()

    # create the thruster devices by specifying the thruster type and its location and direction
    for pos_B, dir_B in zip(thruster_location, thrust_direction):
        thruster_factory.create("THRUSTME_NTP30", pos_B, dir_B)

    # get number of thruster devices
    num_thruster = thruster_factory.getNumOfDevices()

    # create thruster object container and tie to spacecraft object
    thruster_model_tag = "ACSThrusterDynamics"
    thruster_factory.addToSpacecraft(thruster_model_tag, thruster_dynamics, deputy_obj)

    # extForceTorque acting as the thruster force
    deputy_ext_force_obj = extForceTorque.ExtForceTorque()
    deputy_ext_force_obj.ModelTag = "external_dist"
    deputy_obj.addDynamicEffector(deputy_ext_force_obj)
    sc_sim.AddModelToTask(dyn_task_name, deputy_ext_force_obj, 3)

    # simple nav acting as the onboard sensors
    chief_nav_obj = simpleNav.SimpleNav()
    chief_nav_obj.scStateInMsg.subscribeTo(chief_obj.scStateOutMsg)
    sc_sim.AddModelToTask(dyn_task_name, chief_nav_obj, 1)

    deputy_nav_obj = simpleNav.SimpleNav()
    deputy_nav_obj.scStateInMsg.subscribeTo(deputy_obj.scStateOutMsg)
    sc_sim.AddModelToTask(dyn_task_name, deputy_nav_obj, 1)

    # flight software process and task
    fsw_process_name = "fsw_process"
    fsw_task_name = "fswTask"
    fsw_process = sc_sim.CreateNewProcess(fsw_process_name, 1)
    fsw_process.addTask(sc_sim.CreateNewTask(fsw_task_name, macros.sec2nano(15.0)))

    # creating vehicle configuration message for attitude control model
    vehicle_config = messaging.vehicleConfigMsgPayload()
    vehicle_config.ISCPntB_B = I
    vehicle_config_msg = messaging.vehicleConfigMsg().write(vehicle_config)

    # alMeanOEFeedback
    mean_oe_feedback_obj = alMeanOEFeedback.alMeanOEFeedback()
    mean_oe_feedback_obj.ModelTag = "alMeanOEFeedback"
    mean_oe_feedback_obj.chiefTransInMsg.subscribeTo(chief_nav_obj.transOutMsg)
    mean_oe_feedback_obj.deputyTransInMsg.subscribeTo(deputy_nav_obj.transOutMsg)
    mean_oe_feedback_obj.deputyAttNavInMsg.subscribeTo(deputy_nav_obj.attOutMsg)

    # deputy input force input message
    deputy_ext_force_obj.cmdForceInertialInMsg.subscribeTo(
        mean_oe_feedback_obj.forceOutMsg
    )

    mean_oe_feedback_obj.K = [
        1e7,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1e7,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1e7,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1e7,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1e7,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1e7,
    ]
    mean_oe_feedback_obj.max_thrust = max_thrust  # [N]

    mean_oe_feedback_obj.mu = orbitalMotion.MU_EARTH * 1e9  # [m^3/s^2]
    mean_oe_feedback_obj.req = orbitalMotion.REQ_EARTH * 1e3  # [m]
    mean_oe_feedback_obj.J2 = orbitalMotion.J2_EARTH

    mean_oe_feedback_obj.phase_angle = phase_angle  # [rad]
    mean_oe_feedback_obj.ring_height = ring_height  # [m]
    mean_oe_feedback_obj.ring_diameter = ring_diameter  # [m]

    mean_oe_feedback_obj.windowDeg_apses = (
        degree_window_apses  # [deg], half-width of the thrust windows
    )
    mean_oe_feedback_obj.windowDeg_nodes = (
        degree_window_nodes  # [deg], half-width of the thrust windows
    )

    mean_oe_feedback_obj.enableNodeWindows = 1
    mean_oe_feedback_obj.enableApsesWindows = 1

    mean_oe_feedback_obj.varying_target = (
        1  # 1 means to use formation guidance, 0 uses target difference
    )
    mean_oe_feedback_obj.targetDiffOeMean = [
        0.000,
        0.000253,
        0.000253,
        0.000207,
        0.000207,
        0.000,
    ]  # not used if varying_target = 1, [da, dEx, dEy, dIx, dIy, dLambda]

    sc_sim.AddModelToTask(fsw_task_name, mean_oe_feedback_obj, 1)

    # set up flight software algorithm tasks
    inertial_3d = inertial3D.inertial3D()
    inertial_3d.ModelTag = "inertial3D"
    inertial_3d.sigma_R0N = [0.0, 0.0, 0.0]  # inertial frame
    sc_sim.AddModelToTask(fsw_task_name, inertial_3d)

    # attitude tracking error
    attitude_tracking_error = attTrackingError.attTrackingError()
    attitude_tracking_error.ModelTag = "attitude_tracking_error"
    attitude_tracking_error.attRefInMsg.subscribeTo(inertial_3d.attRefOutMsg)
    attitude_tracking_error.attNavInMsg.subscribeTo(deputy_nav_obj.attOutMsg)
    sc_sim.AddModelToTask(fsw_task_name, attitude_tracking_error)

    # MRP attitude feedback control
    mrp_feedback = mrpFeedback.mrpFeedback()
    mrp_feedback.ModelTag = "mrp_feedback"
    sc_sim.AddModelToTask(fsw_task_name, mrp_feedback)
    mrp_feedback.K = 3.5 * 10
    mrp_feedback.Ki = 0.0002
    mrp_feedback.P = 30.0 * 10
    mrp_feedback.integralLimit = 2 / mrp_feedback.Ki * 0.1
    mrp_feedback.attGuidInMsg.subscribeTo(attitude_tracking_error.attGuidOutMsg)
    mrp_feedback.vehConfigInMsg.subscribeTo(vehicle_config_msg)

    # set up thruster force mapping module
    thruster_force_mapping = thrForceMapping.thrForceMapping()
    thruster_force_mapping.ModelTag = "thruster_force_mapping"
    sc_sim.AddModelToTask(fsw_task_name, thruster_force_mapping)

    control_axes_B = [-1, 0, 0]  # control along body -x-axis
    thruster_force_mapping.thrForceSign = -1

    # set up schmitt trigger thruster firing logic
    thrust_firing_schmitt = thrFiringSchmitt.thrFiringSchmitt()
    thrust_firing_schmitt.ModelTag = "thrust_firing_schmitt"
    sc_sim.AddModelToTask(fsw_task_name, thrust_firing_schmitt)
    thrust_firing_schmitt.thrMinFireTime = 0.002
    thrust_firing_schmitt.level_on = 0.75
    thrust_firing_schmitt.level_off = 0.25

    thruster_dynamics.cmdsInMsg.subscribeTo(thrust_firing_schmitt.thrCmdsOutMsg)

    # chief initial conditions
    chief_oe = orbitalMotion.ClassicElements()
    chief_oe.a = chief_a  # [m]
    chief_oe.e = chief_e
    chief_oe.i = chief_i  # [rad]
    chief_oe.Omega = chief_Omega  # [rad]
    chief_oe.omega = chief_omega  # [rad]
    chief_oe.f = chief_f  # [rad]

    # orbital elements to position and velocity
    chief_rN, chief_vN = orbitalMotion.elem2rv(mu, chief_oe)
    orbitalMotion.rv2elem(mu, chief_rN, chief_vN)
    chief_obj.hub.r_CN_NInit = chief_rN  # [m]
    chief_obj.hub.v_CN_NInit = chief_vN  # [m/s]

    # deputy initial conditions
    deputy_oe = orbitalMotion.ClassicElements()
    deputy_oe.a = deputy_a
    deputy_oe.e = deputy_e
    deputy_oe.i = deputy_i  # [rad]
    deputy_oe.Omega = deputy_Omega  # [rad]
    deputy_oe.omega = deputy_omega  # [rad]
    deputy_oe.f = deputy_f

    # orbital elements to position and velocity
    deputy_rN, deputy_vN = orbitalMotion.elem2rv(mu, deputy_oe)
    deputy_obj.hub.r_CN_NInit = deputy_rN  # [m]
    deputy_obj.hub.v_CN_NInit = deputy_vN  # [m/s]

    # logging output messages
    orbit_period = 2 * math.pi / math.sqrt(mu / chief_oe.a**3)
    simulation_time = orbit_period * numOrbits
    simulation_time = macros.sec2nano(simulation_time)
    num_data_points = 1000
    sampling_time = unitTestSupport.samplingTime(
        simulation_time, dyn_time_step, num_data_points
    )

    # record output messages
    chief_data_log = chief_obj.scStateOutMsg.recorder(sampling_time)
    deputy_data_log = deputy_obj.scStateOutMsg.recorder(sampling_time)
    ext_force_data_log = mean_oe_feedback_obj.forceOutMsg.recorder(sampling_time)

    sc_sim.AddModelToTask(dyn_task_name, chief_data_log)
    sc_sim.AddModelToTask(dyn_task_name, deputy_data_log)
    sc_sim.AddModelToTask(dyn_task_name, ext_force_data_log)

    # execute the simulation
    sc_sim.InitializeSimulation()
    sc_sim.ConfigureStopTime(simulation_time)
    sc_sim.ExecuteSimulation()

    # pull logged data
    chief_rN = chief_data_log.r_BN_N
    chief_vN = chief_data_log.v_BN_N
    deputy_rN = deputy_data_log.r_BN_N
    deputy_vN = deputy_data_log.v_BN_N

    deputy_mrp_data = deputy_data_log.sigma_BN
    time_data = chief_data_log.times() * macros.NANO2SEC / orbit_period
    inertial_force = (
        ext_force_data_log.forceRequestInertial
    )  #  [N] control force requested

    body_force_B = np.empty(inertial_force.shape)
    chief_oed_eq = np.empty((len(chief_rN[:, 0]), 6))
    chief_oed_eq_err = np.empty((len(chief_rN[:, 0]), 6))

    for i in range(0, len(chief_rN[:, 0])):
        # chief spacecraft
        # convert chief spacecraft state to orbital elements
        chief_oe_cl_osc = orbitalMotion.rv2elem(mu, chief_rN[i], chief_vN[i])

        # map chief element to mean orbital elements
        chief_oe_cl_mean = orbitalMotion.ClassicElements()
        orbitalMotion.clMeanOscMap(
            orbitalMotion.REQ_EARTH * 1e3,
            orbitalMotion.J2_EARTH,
            chief_oe_cl_osc,
            chief_oe_cl_mean,
            -1,
        )

        # convert chief mean orbital elements to equinoctial elements
        chief_oe_eq_mean = orbitalMotion.EquinoctialElements()
        orbitalMotion.clElem2eqElem(chief_oe_cl_mean, chief_oe_eq_mean)

        # deputy spacecraft
        # convert deputy spacecraft state to mean orbital elements
        deputy_oe_cl_osc = orbitalMotion.rv2elem(mu, deputy_rN[i], deputy_vN[i])

        # map deputy element to mean orbital elements
        deputy_oe_cl_mean = orbitalMotion.ClassicElements()
        orbitalMotion.clMeanOscMap(
            orbitalMotion.REQ_EARTH * 1e3,
            orbitalMotion.J2_EARTH,
            deputy_oe_cl_osc,
            deputy_oe_cl_mean,
            -1,
        )

        # convert deputy mean orbital elements to equinoctial elements
        deputy_oe_eq_mean = orbitalMotion.EquinoctialElements()
        orbitalMotion.clElem2eqElem(deputy_oe_cl_mean, deputy_oe_eq_mean)

        # calculate relative orbital elements
        delta_i_mag = ring_height / (2 * chief_oe_cl_osc.a)
        delta_e_mag = ring_diameter / (2 * chief_oe_cl_osc.a)

        delta_i_x = delta_i_mag * np.cos(phase_angle)
        delta_i_y = delta_i_mag * np.sin(phase_angle)

        delta_e_x = delta_e_mag * np.cos(phase_angle)
        delta_e_y = delta_e_mag * np.sin(phase_angle)

        e2 = chief_oe_cl_osc.e * chief_oe_cl_osc.e
        gamma = (
            -1.5
            * orbitalMotion.J2_EARTH
            * ((orbitalMotion.REQ_EARTH * 1e3) / chief_oe_cl_osc.a)
            * np.sqrt(1.0 / np.pow(1.0 - e2, 4))
        )
        delta_a = -7.0 * gamma * np.sin(2.0 * chief_oe_cl_osc.i) * delta_i_x

        # calculate oed error
        chief_oed_eq_err[i, 0] = (
            deputy_oe_eq_mean.a - chief_oe_eq_mean.a
        ) / chief_oe_eq_mean.a - delta_a  # delta a (normalized)
        chief_oed_eq_err[i, 1] = (
            deputy_oe_eq_mean.P1 - chief_oe_eq_mean.P1 - delta_e_x
        )  # delta P1
        chief_oed_eq_err[i, 2] = (
            deputy_oe_eq_mean.P2 - chief_oe_eq_mean.P2 - delta_e_y
        )  # delta P2
        chief_oed_eq_err[i, 3] = (
            deputy_oe_eq_mean.Q1 - chief_oe_eq_mean.Q1 - delta_i_x
        )  # delta Q1
        chief_oed_eq_err[i, 4] = (
            deputy_oe_eq_mean.Q2 - chief_oe_eq_mean.Q2 - delta_i_y
        )  # delta Q2
        chief_oed_eq_err[i, 5] = deputy_oe_eq_mean.l - chief_oe_eq_mean.l  # delta l

        # calculate oed
        chief_oed_eq[i, 0] = (
            deputy_oe_eq_mean.a - chief_oe_eq_mean.a
        ) / chief_oe_eq_mean.a  # delta a (normalized)
        chief_oed_eq[i, 1] = deputy_oe_eq_mean.P1 - chief_oe_eq_mean.P1  # delta P1
        chief_oed_eq[i, 2] = deputy_oe_eq_mean.P2 - chief_oe_eq_mean.P2  # delta P2
        chief_oed_eq[i, 3] = deputy_oe_eq_mean.Q1 - chief_oe_eq_mean.Q1  # delta Q1
        chief_oed_eq[i, 4] = deputy_oe_eq_mean.Q2 - chief_oe_eq_mean.Q2  # delta Q2
        chief_oed_eq[i, 5] = deputy_oe_eq_mean.l - chief_oe_eq_mean.l  # delta l
        while chief_oed_eq[i, 5] > math.pi:
            chief_oed_eq[i, 5] = chief_oed_eq[i, 5] - 2 * math.pi
        while chief_oed_eq[i, 5] < -math.pi:
            chief_oed_eq[i, 5] = chief_oed_eq[i, 5] + 2 * math.pi
        chief_oed_eq_err[i, 5] = chief_oed_eq[i, 5]

        # calculate body force
        dcmBN = RigidBodyKinematics.MRP2C(deputy_mrp_data[i])
        body_force_B[i, :] = np.dot(dcmBN, inertial_force[i, :])

    ### PLOTS
    fig, axs = plt.subplots(3, 2, figsize=(10, 8), sharex=True)

    labels = [
        r"$\Delta a_{err}$",
        r"$\Delta e_{x,err}$",
        r"$\Delta e_{y,err}$",
        r"$\Delta i_{x,err}$",
        r"$\Delta i_{y,err}$",
        r"$\Delta \lambda_{err}$",
    ]

    for i in range(6):
        row = i // 2
        col = i % 2
        axs[row, col].plot(time_data, chief_oed_eq_err[:, i], label=labels[i])
        axs[row, col].legend(loc="best")
        axs[row, col].grid(True)
        axs[row, col].set_ylabel(f"{labels[i]} MOE Error")

    axs[-1, 0].set_xlabel("time [orbit]")
    axs[-1, 1].set_xlabel("time [orbit]")

    fig.suptitle("MOE Error Components", fontsize=14)
    plt.tight_layout(rect=[0, 0, 1, 0.97])

    ## ΔV BUDGET AND USAGE
    # using commanded inertial force
    mass = float(deputy_obj.hub.mHub)  # [kg]
    t_sec = deputy_data_log.times() * macros.NANO2SEC
    dt = np.diff(t_sec, prepend=t_sec[0])  # [s]

    force_mag = np.linalg.norm(inertial_force, axis=1)  # [N]

    # set a threshold to avoid numerical noise when thrust is effectively off
    THR_THRESH = 1e-11  # [N]
    accel_mag = np.where(force_mag > THR_THRESH, force_mag / mass, 0.0)  # [m/s^2]

    # compute ΔV
    dv_inst = accel_mag  # [m/s]
    dv_cum = np.cumsum(dv_inst * dt)  # [m/s]
    total_dv = dv_cum[-1]  # [m/s]
    avg_dv_per_orbit = total_dv / numOrbits  # [m/s]
    peak_force = float(np.max(force_mag))  # [N]

    ## PRINT SUMMARY
    print("\n=== ΔV Usage Analysis ===")
    print(
        f"Altitude: {altitude:.0f} km, Cross-track: {ring_diameter/1e3:.0f} km, Phase: {phase_angle:.1f}°"
    )
    print(f"Total ΔV required (0–{numOrbits:d} orbits):     {total_dv:.4f} m/s")
    print(f"Average ΔV per orbit:        {avg_dv_per_orbit:.4f} m/s/orbit")
    print(f"Peak commanded force:       {peak_force:.6e} N")

    # plot commanded force magnitude
    plt.figure(5)
    plt.plot(time_data, force_mag, label="|F| (N)")
    plt.xlabel("time [orbit]")
    plt.ylabel("Force [N]")
    plt.title("Commanded Force Magnitude")
    plt.grid(True)
    plt.legend()

    # plot cumulative ΔV
    plt.figure(6)
    plt.plot(time_data, dv_cum, label="Cumulative ΔV")
    plt.xlabel("time [orbit]")
    plt.ylabel("ΔV [m/s]")
    plt.title("Cumulative ΔV to Maintain Formation")
    plt.grid(True)
    plt.legend()

    plt.show()


if __name__ == "__main__":
    run(45)  # number of orbits
