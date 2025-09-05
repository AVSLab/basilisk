import math
import os

import matplotlib.pyplot as plt
import numpy as np
from Basilisk import __path__
from Basilisk.fswAlgorithms import attitudeMOEFeedback
from Basilisk.simulation import extForceTorque
from Basilisk.simulation import simpleNav
from Basilisk.simulation import spacecraft
from Basilisk.utilities import SimulationBaseClass
from Basilisk.simulation import GravityGradientEffector
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import RigidBodyKinematics
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import unitTestSupport


bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


def run(numOrbits):

    # mission parameters
    sat_mass = 60  # [kg]
    I = [4.42, 0.0, 0.0, 0.0, 5.59, 0.0, 0.0, 0.0, 9.69]

    degree_window = 15  # [deg], half-width of the thrust windows

    # formation parameters
    altitude = 600  # [km]
    phase_angle = 45  # [rad]
    ring_diameter = 10 * 1e3  # [m]
    ring_height = ring_diameter / 2  # [m]

    # chief_a = 6978137.000  # [m]
    # chief_e = 0.00727
    # chief_i = np.deg2rad(98.0)  # [rad]
    # chief_Omega = 0.0  # [rad]
    # chief_omega = 0.0  # [rad]
    # chief_f = 0  # [rad]

    # deputy_a = 6978140.085
    # deputy_e = 0.00753
    # deputy_i = np.deg2rad(98.02903)  # [rad]
    # deputy_Omega = np.deg2rad(0.02931)  # [rad]
    # deputy_omega = np.deg2rad(1.92871)  # [rad]
    # deputy_f = np.deg2rad(360 - 1.96836)

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
    max_thrust = 0.002  # [N]
    Isp_sec = 2400  #  NPT30 [s]
    m_prop = 1.0  #  [kg]

    sc_sim = SimulationBaseClass.SimBaseClass()
    sc_sim.SetProgressBar(True)

    # dynamics
    dynProcessName = "dynProcess"
    dyn_task_name = "dynTask"
    dynProcess = sc_sim.CreateNewProcess(dynProcessName, 2)
    dyn_time_step = macros.sec2nano(15.0)
    dynProcess.addTask(sc_sim.CreateNewTask(dyn_task_name, dyn_time_step))

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

    # extObj acting as the thruster force
    deputy_ext_force_obj = extForceTorque.ExtForceTorque()
    deputy_ext_force_obj.ModelTag = "external_dist"
    deputy_obj.addDynamicEffector(deputy_ext_force_obj)
    sc_sim.AddModelToTask(dyn_task_name, deputy_ext_force_obj, 3)

    # simple nav acting as the onboard sensors
    chief_nav_obj = simpleNav.SimpleNav()
    deputy_nav_obj = simpleNav.SimpleNav()
    chief_nav_obj.scStateInMsg.subscribeTo(chief_obj.scStateOutMsg)
    deputy_nav_obj.scStateInMsg.subscribeTo(deputy_obj.scStateOutMsg)
    sc_sim.AddModelToTask(dyn_task_name, chief_nav_obj, 1)
    sc_sim.AddModelToTask(dyn_task_name, deputy_nav_obj, 1)

    # FSW
    fsw_process_name = "fsw_process"
    fsw_task_name = "fswTask"
    fsw_process = sc_sim.CreateNewProcess(fsw_process_name, 1)
    fsw_process.addTask(sc_sim.CreateNewTask(fsw_task_name, macros.sec2nano(15.0)))

    # attitudeMOEFeedback
    mean_oe_feedback_obj = attitudeMOEFeedback.attitudeMOEFeedback()
    mean_oe_feedback_obj.ModelTag = "attitudeMOEFeedback"
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
    mean_oe_feedback_obj.J2 = orbitalMotion.J2_EARTH  # []

    mean_oe_feedback_obj.phase_angle = phase_angle  # [rad]
    mean_oe_feedback_obj.ring_height = ring_height  # [m]
    mean_oe_feedback_obj.ring_diameter = ring_diameter  # [m]

    mean_oe_feedback_obj.windowDeg = (
        degree_window  # [deg], half-width of the thrust windows
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

    # Total ΔV from rocket equation
    g0 = 9.80665  # [m/s^2]
    mf = sat_mass - m_prop  # [kg]
    if mf <= 0:
        print(
            "WARNING: Propellant mass is too large, spacecraft mass is zero or negative."
        )
    dv_total_budget = g0 * Isp_sec * np.log(sat_mass / mf)  # [m/s]

    # compute orbital period and orbits per day
    T_orbit = 2 * np.pi / np.sqrt(mu / deputy_oe.a**3)  # [s]
    orbits_per_day = 86400.0 / T_orbit  # [orbits/day]

    # compute consumption rate
    dv_per_orbit_initial = avg_dv_per_orbit  # [m/s/orbit]

    # compute steady-state consumption rate
    idx_ss = int(2 * len(time_data) / 3)  # use last third of simulation

    t_orb = time_data  # [orbits]

    # linear fit dv_cum ~ m * t_orb + v over steady-state section
    m_slope, b_off = np.polyfit(t_orb[idx_ss:], dv_cum[idx_ss:], 1)
    dv_per_orbit_steady = float(
        max(m_slope, 0.0)
    )  # [m/s per orbit], clamp tiny negatives

    # compute duration for each rate
    def duration_from_rate(dv_budget, dv_rate_per_orbit):
        if dv_rate_per_orbit <= 0:
            return np.inf, np.inf
        orbits = dv_budget / dv_rate_per_orbit
        days = orbits / orbits_per_day
        return orbits, days

    orbits_init, days_init = duration_from_rate(dv_total_budget, dv_per_orbit_initial)
    orbits_ss, days_ss = duration_from_rate(dv_total_budget, dv_per_orbit_steady)

    ## PRINT SUMMARY
    print("\n=== ΔV Budget & Maintenance Duration ===")
    print(f"Isp: {Isp_sec:.0f} s, m0: {sat_mass:.2f} kg, m_prop: {m_prop:.3f} kg")
    print(f"Total ΔV budget (rocket equation):               {dv_total_budget:.2f} m/s")
    print(
        f"Avg usage (0-{numOrbits:d} orbits): {dv_per_orbit_initial:.4f} m/s/orbit  "
        f"→ duration ≈ {orbits_init:.1f} orbits ({days_init:.1f} days)"
    )
    print(
        f"Steady-state usage (last 1/3): {dv_per_orbit_steady:.4f} m/s/orbit  "
        f"→ duration ≈ {orbits_ss:.1f} orbits ({days_ss:.1f} days)"
    )

    # plot per orbit usage estimate vs time
    win = max(5, int(0.1 * len(t_orb)))  # ~10% window
    mov_slope = np.empty_like(t_orb)
    mov_slope[:] = np.nan
    for k in range(win, len(t_orb)):
        m_loc, _ = np.polyfit(t_orb[k - win : k], dv_cum[k - win : k], 1)
        mov_slope[k] = m_loc

    plt.figure(7)
    plt.plot(t_orb, mov_slope, label="ΔV usage [m/s per orbit] (moving fit)")
    plt.axhline(dv_per_orbit_initial, linestyle="--", label="Avg (0-end)")
    plt.axhline(dv_per_orbit_steady, linestyle=":", label="Avg (steady-state)")
    plt.xlabel("time [orbit]")
    plt.ylabel("ΔV per orbit [m/s/orbit]")
    plt.title("Instantaneous ΔV Usage (moving linear fit)")
    plt.grid(True)
    plt.legend()

    ## MONTHLY ΔV ESTIMATE FOR GENERAL AND STEADY STATE
    days_per_month = 30.44  # [days/month]
    orbits_per_month = orbits_per_day * days_per_month

    # overall average
    dv_month_general = dv_per_orbit_initial * orbits_per_month  # [m/s per month]

    # steady-state of the last 1/3 of the simulation
    dv_month_steady = dv_per_orbit_steady * orbits_per_month  # [m/s per month]

    print("\n=== Monthly ΔV Estimates ===")
    print(
        f"Orbits/day: {orbits_per_day:.2f}, Orbits/month: {orbits_per_month:.1f} (using {days_per_month:.3f} days/month)"
    )
    print(f"General monthly ΔV (overall avg): {dv_month_general:.2f} m/s per month")
    print(f"Steady-state monthly ΔV:          {dv_month_steady:.2f} m/s per month")

    mov_monthly = mov_slope * orbits_per_month
    plt.figure(8)
    plt.plot(t_orb, mov_monthly, label="Rolling monthly ΔV (from moving slope)")
    plt.axhline(dv_month_general, linestyle="--", label="General monthly ΔV")
    plt.axhline(dv_month_steady, linestyle=":", label="Steady-state monthly ΔV")
    plt.xlabel("time [orbit]")
    plt.ylabel("ΔV per month [m/s]")
    plt.title("Estimated Monthly ΔV Over Time")
    plt.grid(True)
    plt.legend()

    # === PROPELLANT USAGE & TOTAL LIFETIME ===
    # instantaneous mass flow rate
    #   mdot = |F| / (Isp * g0)
    thrust_on = force_mag > THR_THRESH
    mdot_inst = np.where(thrust_on, force_mag / (Isp_sec * g0), 0.0)  # [kg/s]

    # integrate propellant usage over time
    prop_used_cum = np.cumsum(mdot_inst * dt)  # [kg]
    prop_used_total = float(prop_used_cum[-1])  # [kg]
    prop_remaining = max(m_prop - prop_used_total, 0.0)  # [kg], clamp at 0

    # burn-time & duty-cycle stats
    burn_time_sec = float(np.sum(np.where(thrust_on, dt, 0.0)))  # [s]

    # duty cycle = fraction of time spent firing if time is nonzero
    duty_cycle = burn_time_sec / float(t_sec[-1]) if t_sec[-1] > 0 else 0.0

    # average thrust and mdot when thruster is firing
    avg_thrust_when_on = (
        (np.sum(force_mag * dt) / burn_time_sec) if burn_time_sec > 0 else 0.0
    )  # [N]
    avg_mdot_when_on = (
        avg_thrust_when_on / (Isp_sec * g0) if burn_time_sec > 0 else 0.0
    )  # [kg/s]

    # overall-average mdot
    avg_mdot_overall = (
        prop_used_total / float(t_sec[-1]) if t_sec[-1] > 0 else 0.0
    )  # [kg/s]

    # steady-state mdot from last third (robust against initial transients)
    idx_ss = int(2 * len(t_sec) / 3) if len(t_sec) > 0 else 0
    if idx_ss < len(t_sec) - 1:
        mdot_ss = (prop_used_cum[-1] - prop_used_cum[idx_ss]) / (
            t_sec[-1] - t_sec[idx_ss]
        )
        mdot_ss = float(max(mdot_ss, 0.0))  # [kg/s], clamp tiny negatives
    else:
        mdot_ss = avg_mdot_overall

    # lifetime estimates (time to empty)
    def lifetime_seconds(remaining_mass, mdot):
        return float(remaining_mass / mdot) if mdot > 0 else np.inf

    life_overall_sec = lifetime_seconds(prop_remaining, avg_mdot_overall)
    life_ss_sec = lifetime_seconds(prop_remaining, mdot_ss)

    # convenience conversions
    def fmt_time(seconds):
        if not np.isfinite(seconds):
            return "∞"
        days = seconds / 86400.0
        hrs = seconds / 3600.0
        return f"{days:.1f} days ({hrs:.1f} hr)"

    # PRINT SUMMARY
    print("\n=== Propellant Usage & Total Lifetime ===")
    print(f"Propellant available:           {m_prop:.3f} kg")
    print(f"Propellant used in sim:         {prop_used_total:.4f} kg")
    print(f"Propellant remaining:           {prop_remaining:.4f} kg")
    print(
        f"\nTotal burn time this sim:       {burn_time_sec/3600.0:.2f} hr "
        f"(duty cycle {100.0*duty_cycle:.1f}%)"
    )
    print(f"Avg thrust when ON:             {avg_thrust_when_on:.6e} N")
    print(f"Avg mdot when ON:               {avg_mdot_when_on:.6e} kg/s")
    print(f"Overall-average mdot:           {avg_mdot_overall:.6e} kg/s")
    print(f"Steady-state mdot (last 1/3):   {mdot_ss:.6e} kg/s")
    print(
        f"\nEstimated lifetime remaining (overall avg):  {fmt_time(life_overall_sec)}"
    )
    print(f"Estimated lifetime remaining (steady-state): {fmt_time(life_ss_sec)}")

    # PLOTS: cumulative/remaining propellant
    plt.figure(9)
    plt.plot(time_data, prop_used_cum, label="Propellant used [kg]")
    plt.xlabel("time [orbit]")
    plt.ylabel("Propellant used [kg]")
    plt.title("Cumulative Propellant Usage")
    plt.grid(True)
    plt.legend()

    plt.figure(10)
    prop_remaining_series = np.maximum(m_prop - prop_used_cum, 0.0)
    plt.plot(time_data, prop_remaining_series, label="Propellant remaining [kg]")
    plt.xlabel("time [orbit]")
    plt.ylabel("Propellant remaining [kg]")
    plt.title("Propellant Remaining vs Time")
    plt.grid(True)
    plt.legend()

    # plt.show()

    def thrust_time_per_orbit(t_sec, dt, force_mag, T_orbit, thr_thresh=0.0) -> tuple:
        """
        Compute total thrust-on time and duty cycle in each orbit.

        Args:
        t_sec      : time [s]
        dt         : time step [s]
        force_mag  : thrust magnitude [N]
        T_orbit    : orbit period [s]
        thr_thresh : threshold [N] below which thrust is considered off

        Returns:
        thrust_time_s   : 1D array [num_orbits] of total thrust-on time (s) in each orbit
        duty_cycle      : 1D array [num_orbits] of thrust_time_s / T_orbit
        orbit_edges_sec : 1D array [num_orbits+1] orbit boundary times (s)
        """
        # time since sim start
        t0 = float(t_sec[0])
        t_rel = t_sec - t0

        # orbit index for each sample (0,1,2,...)
        orbit_idx = np.floor(t_rel / T_orbit).astype(int)
        num_orbits_int = int(np.max(orbit_idx)) + 1

        # thrust-on mask
        thrust_on = force_mag > thr_thresh

        # accumulate thrust-on dt per orbit
        thrust_time_s = np.zeros(num_orbits_int)
        for k in range(num_orbits_int):
            mask = orbit_idx == k
            if np.any(mask):
                thrust_time_s[k] = np.sum(np.where(thrust_on[mask], dt[mask], 0.0))

        duty_cycle = thrust_time_s / T_orbit

        # orbit boundary times (for reference/plotting)
        orbit_edges_sec = t0 + np.arange(num_orbits_int + 1) * T_orbit
        return thrust_time_s, duty_cycle, orbit_edges_sec

    thrust_time_s, duty_cycle, orbit_edges_sec = thrust_time_per_orbit(
        t_sec=t_sec, dt=dt, force_mag=force_mag, T_orbit=T_orbit, thr_thresh=THR_THRESH
    )

    # quick summary
    print("\n=== Thrust Time Per Orbit ===")
    print(f"Computed over {len(thrust_time_s)} full orbits")
    print(
        f"Mean thrust time/orbit: {np.mean(thrust_time_s):.2f} s  "
        f"(mean duty cycle {100*np.mean(duty_cycle):.2f}%)"
    )
    print(
        f"Max  thrust time/orbit: {np.max(thrust_time_s):.2f} s  "
        f"(max  duty cycle {100*np.max(duty_cycle):.2f}%)"
    )

    #  thrust time per orbit
    plt.figure()
    plt.plot(np.arange(len(thrust_time_s)), thrust_time_s, marker="o")
    plt.xlabel("Orbit index")
    plt.ylabel("Thrust-on time per orbit [s]")
    plt.title("Total Thrust Time Per Orbit")
    plt.grid(True)

    # # === Elements at ~100 orbits ===
    # # build a physical time vector [s]
    # t_sec = chief_data_log.times() * macros.NANO2SEC

    # # orbit period (use chief's initial 'a')
    # T_orbit = 2 * np.pi / np.sqrt(mu / chief_oe.a**3)  # [s]
    # t_target = 150.0 * T_orbit  # [s]

    # # find the sample closest to 100 orbits
    # idx_100 = int(np.argmin(np.abs(t_sec - t_target)))
    # orbits_at_idx = t_sec[idx_100] / T_orbit

    # # helper: print classical elements nicely
    # def _print_cl(tag, osc, mean):
    #     def deg(x):
    #         return np.rad2deg(x)

    #     print(
    #         f"\n{tag} @ ~100 orbits (sample {idx_100}, t = {orbits_at_idx:.2f} orbits):"
    #     )
    #     print("  Osculating (CL):")
    #     print(f"    {tag}_a     = {osc.a:12.3f} # [m]")
    #     print(f"    {tag}_e     = {osc.e:12.8f}")
    #     print(f"    {tag}_i     = np.deg2rad({deg(osc.i):12.6f})  # [deg]")
    #     print(f"    {tag}_Omega  = np.deg2rad({deg(osc.Omega):12.6f}) # [deg]")
    #     print(f"    {tag}_omega = np.deg2rad({deg(osc.omega):12.6f}) # [deg]")
    #     print(f"    {tag}_f     = np.deg2rad({deg(osc.f):12.6f}) # [deg]")

    # # compute chief elements at idx_100
    # chief_osc_100 = orbitalMotion.rv2elem(mu, chief_rN[idx_100], chief_vN[idx_100])
    # chief_mean_100 = orbitalMotion.ClassicElements()
    # orbitalMotion.clMeanOscMap(
    #     orbitalMotion.REQ_EARTH * 1e3,
    #     orbitalMotion.J2_EARTH,
    #     chief_osc_100,
    #     chief_mean_100,
    #     -1,
    # )

    # # compute deputy elements at idx_100
    # deputy_osc_100 = orbitalMotion.rv2elem(mu, deputy_rN[idx_100], deputy_vN[idx_100])
    # deputy_mean_100 = orbitalMotion.ClassicElements()
    # orbitalMotion.clMeanOscMap(
    #     orbitalMotion.REQ_EARTH * 1e3,
    #     orbitalMotion.J2_EARTH,
    #     deputy_osc_100,
    #     deputy_mean_100,
    #     -1,
    # )

    # # print
    # _print_cl("chief", chief_osc_100, chief_mean_100)
    # _print_cl("deputy", deputy_osc_100, deputy_mean_100)

    plt.show()


if __name__ == "__main__":
    run(425)  # number of orbits
