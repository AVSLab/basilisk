#
#  ISC License
#
#  Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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


import math
import os

import matplotlib.pyplot as plt
import numpy as np
from Basilisk import __path__
from Basilisk.fswAlgorithms import meanOEFeedback
from Basilisk.simulation import extForceTorque
from Basilisk.simulation import simpleNav
from Basilisk.simulation import spacecraft
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import RigidBodyKinematics
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import unitTestSupport


bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


def run(numOrbits):

    sc_sim = SimulationBaseClass.SimBaseClass()
    sc_sim.SetProgressBar(True)

    # dynamics
    dynProcessName = "dynProcess"
    dyn_task_name = "dynTask"
    dynProcess = sc_sim.CreateNewProcess(dynProcessName, 2)
    dyn_time_step = macros.sec2nano(15.0)
    dynProcess.addTask(sc_sim.CreateNewTask(dyn_task_name, dyn_time_step))

    # sc
    I = [4.42, 0.0, 0.0, 0.0, 5.59, 0.0, 0.0, 0.0, 9.69]

    chief_obj = spacecraft.Spacecraft()
    chief_obj.ModelTag = "chief_obj"
    chief_obj.hub.mHub = 60.0
    chief_obj.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    chief_obj.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)
    sc_sim.AddModelToTask(dyn_task_name, chief_obj, 2)

    deputy_obj = spacecraft.Spacecraft()
    deputy_obj.ModelTag = "deputy_obj"
    deputy_obj.hub.mHub = 60.0
    deputy_obj.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    deputy_obj.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)
    sc_sim.AddModelToTask(dyn_task_name, deputy_obj, 2)

    # grav
    grav_factor = simIncludeGravBody.gravBodyFactory()
    earth = grav_factor.createEarth()
    earth.isCentralBody = True
    mu = earth.mu
    earth.useSphericalHarmonicsGravityModel(
        bskPath + "/supportData/LocalGravData/GGM03S.txt", 2
    )
    grav_factor.addBodiesTo(chief_obj)
    grav_factor.addBodiesTo(deputy_obj)

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

    # meanOEFeedback
    mean_oe_feedback_obj = meanOEFeedback.meanOEFeedback()
    mean_oe_feedback_obj.ModelTag = "meanOEFeedback"
    mean_oe_feedback_obj.chiefTransInMsg.subscribeTo(chief_nav_obj.transOutMsg)
    mean_oe_feedback_obj.deputyTransInMsg.subscribeTo(deputy_nav_obj.transOutMsg)

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

    # Used for static target difference. These are initial values for algorithm validation,
    # but the formation is re-calculated at each time step using the mean orbital elements.
    mean_oe_feedback_obj.targetDiffOeMean = [
        0.0,
        0.000255,
        0.000255,
        0.000510,
        0.000510,
        0.00,
    ]

    # use equinoctial elements
    mean_oe_feedback_obj.oeType = 1

    # formation parameters
    phase_angle = 45  # [rad]
    ring_height = 10 * 1e3  # [m]
    ring_diameter = 5 * 1e3  # [m]

    # create the meanOEFeedback object
    mean_oe_feedback_obj.mu = orbitalMotion.MU_EARTH * 1e9  # [m^3/s^2]
    mean_oe_feedback_obj.req = orbitalMotion.REQ_EARTH * 1e3  # [m]
    mean_oe_feedback_obj.J2 = orbitalMotion.J2_EARTH  # []
    mean_oe_feedback_obj.varying_target = 1
    mean_oe_feedback_obj.phase_angle = phase_angle  # [rad]
    mean_oe_feedback_obj.ring_height = ring_height  # [m]
    mean_oe_feedback_obj.ring_diameter = ring_diameter  # [m]

    sc_sim.AddModelToTask(fsw_task_name, mean_oe_feedback_obj, 1)

    # # spacecraft initial conditions
    # chief_oe = orbitalMotion.ClassicElements()
    # chief_oe.a = 6928137  # [m]
    # chief_oe.e = 0.000727
    # chief_oe.i = np.deg2rad(98.0)  # [rad]
    # chief_oe.Omega = 0.0  # [rad]
    # chief_oe.omega = 0.0  # [rad]
    # chief_oe.f = 0  # [rad]

    # # orbital elements to position and velocity
    # chief_rN, chief_vN = orbitalMotion.elem2rv(mu, chief_oe)
    # orbitalMotion.rv2elem(mu, chief_rN, chief_vN)
    # chief_obj.hub.r_CN_NInit = chief_rN  # [m]
    # chief_obj.hub.v_CN_NInit = chief_vN  # [m/s]

    # deputy_oe = orbitalMotion.ClassicElements()
    # deputy_oe.a = 6928140.130
    # # deputy_oe.e = 0.000753
    # deputy_oe.e = 0.000953
    # deputy_oe.i = np.deg2rad(98.02924)  # [rad]
    # deputy_oe.Omega = np.deg2rad(0.02953)  # [rad]
    # deputy_oe.omega = np.deg2rad(1.94215)  # [rad]
    # deputy_oe.f = np.deg2rad(358.01792)

    ##
    print("500 km w/ 18 km cross-track separation")

    # spacecraft initial conditions
    chief_oe = orbitalMotion.ClassicElements()
    chief_oe.a = 6878137.000  # [m]
    chief_oe.e = 0.00727
    chief_oe.i = np.deg2rad(98.0)  # [rad]
    chief_oe.Omega = 0.0  # [rad]
    chief_oe.omega = 0.0  # [rad]
    chief_oe.f = 0  # [rad]

    # orbital elements to position and velocity
    chief_rN, chief_vN = orbitalMotion.elem2rv(mu, chief_oe)
    orbitalMotion.rv2elem(mu, chief_rN, chief_vN)
    chief_obj.hub.r_CN_NInit = chief_rN  # [m]
    chief_obj.hub.v_CN_NInit = chief_vN  # [m/s]

    deputy_oe = orbitalMotion.ClassicElements()
    deputy_oe.a = 6878141.763
    deputy_oe.e = 0.00775
    deputy_oe.i = np.deg2rad(98.05301)  # [rad]
    deputy_oe.Omega = np.deg2rad(0.05353)  # [rad]
    deputy_oe.omega = np.deg2rad(3.42403)  # [rad]
    deputy_oe.f = np.deg2rad(360 - 3.49644)

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

    body_force = ext_force_data_log.forceRequestInertial  #  [N] Control force requested
    body_force_B = np.empty(body_force.shape)

    chief_oed_eq = np.empty((len(chief_rN[:, 0]), 6))
    chief_oed_eq_err = np.empty((len(chief_rN[:, 0]), 6))
    for i in range(0, len(chief_rN[:, 0])):
        # chief spacecraft
        # convert chief spacecraft state to mean orbital elements
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
        body_force_B[i, :] = np.dot(dcmBN, body_force[i, :])

    plt.figure(1)
    plt.plot(time_data, chief_oed_eq[:, 0], label="da")
    plt.plot(time_data, chief_oed_eq[:, 1], label="dP1")
    plt.plot(time_data, chief_oed_eq[:, 2], label="dP2")
    plt.plot(time_data, chief_oed_eq[:, 3], label="dQ1")
    plt.plot(time_data, chief_oed_eq[:, 4], label="dQ2")
    plt.plot(time_data, chief_oed_eq[:, 5], label="dl")
    plt.legend()
    plt.xlabel("time [orbit]")
    plt.ylabel("MOE difference")
    plt.title("Mean Orbital Element Differences")

    plt.figure(3)
    plt.plot(time_data, body_force_B[:, 0], label="x")
    plt.plot(time_data, body_force_B[:, 1], label="y")
    plt.plot(time_data, body_force_B[:, 2], label="z")
    plt.legend()
    plt.xlabel("time [orbit]")
    plt.ylabel("Thruster Force [N]")
    plt.title("Commanded Body Force")

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

    # ΔV BUDGET AND USAGE

    # This section computes the total ΔV required to maintain formation,
    # Use commanded inertial force; ΔV = ∫ ||F||/m dt
    mass = float(deputy_obj.hub.mHub)  # [kg]
    t_sec = chief_data_log.times() * macros.NANO2SEC
    dt = np.diff(t_sec, prepend=t_sec[0])  # [s]

    force_inertial = body_force  # [N]
    force_mag = np.linalg.norm(force_inertial, axis=1)  # [N]

    # Threshold to suppress numerical noise when thrust is effectively off
    THR_THRESH = 1e-11  # [N]
    accel_mag = np.where(force_mag > THR_THRESH, force_mag / mass, 0.0)  # [m/s^2]

    dv_inst = accel_mag * dt  # [m/s] per step
    dv_cum = np.cumsum(dv_inst)  # [m/s]
    total_dv = float(dv_cum[-1])
    avg_dv_per_orbit = total_dv / numOrbits
    peak_force = float(force_mag.max())

    # Print summary
    print(f"Total ΔV to maintain formation: {total_dv:.4f} m/s over {numOrbits} orbits")
    print(f"Average ΔV per orbit:         {avg_dv_per_orbit:.4f} m/s/orbit")
    print(f"Peak commanded force:         {peak_force:.6e} N")

    # Plot commanded force magnitude
    plt.figure(5)
    plt.plot(time_data, force_mag, label="|F| (N)")
    plt.xlabel("time [orbit]")
    plt.ylabel("Force [N]")
    plt.title("Commanded Force Magnitude")
    plt.grid(True)
    plt.legend()

    # Plot cumulative ΔV
    plt.figure(6)
    plt.plot(time_data, dv_cum, label="Cumulative ΔV")
    plt.xlabel("time [orbit]")
    plt.ylabel("ΔV [m/s]")
    plt.title("Cumulative ΔV to Maintain Formation")
    plt.grid(True)
    plt.legend()

    """
    THRUSTER SPECS - From Carlo's email using H2 mode 
    average thruster force: 1.0e-3 N each
    steady-state thruster force: 1.1 +/-0.05 mN (one thrusters)
    isp 2500 s
    """
    g0 = 9.80665  # [m/s^2]
    Isp_sec = 2400  #  NPT30 Isp here [s]
    m0 = float(deputy_obj.hub.mHub)  # [kg] total mass at start
    m_prop = 2.4  #  [kg]

    # Total ΔV budget from rocket equation
    mf = m0 - m_prop
    if mf <= 0:
        raise ValueError("Propellant mass exceeds initial mass.")
    dv_total_budget = g0 * Isp_sec * np.log(m0 / mf)  # [m/s]

    # Orbit period and orbits/day
    T_orbit = 2 * np.pi / np.sqrt(mu / chief_oe.a**3)  # [s]
    orbits_per_day = 86400.0 / T_orbit

    # Consumption rates:
    dv_per_orbit_initial = avg_dv_per_orbit  # [m/s per orbit]

    # Robust steady-state rate: fit slope of dv_cum vs time over last third of data
    idx_ss = int(2 * len(time_data) / 3)

    # time_data is in orbits; convert to seconds to avoid unit confusion (optional)
    t_orb = time_data  # [orbits]

    # Linear fit dv_cum ~ m * t_orb + b over steady-state section
    m_slope, b_off = np.polyfit(t_orb[idx_ss:], dv_cum[idx_ss:], 1)
    dv_per_orbit_steady = float(
        max(m_slope, 0.0)
    )  # [m/s per orbit], clamp tiny negatives

    # Durations for each rate
    def duration_from_rate(dv_budget, dv_rate_per_orbit):
        if dv_rate_per_orbit <= 0:
            return np.inf, np.inf
        orbits = dv_budget / dv_rate_per_orbit
        days = orbits / orbits_per_day
        return orbits, days

    orbits_init, days_init = duration_from_rate(dv_total_budget, dv_per_orbit_initial)
    orbits_ss, days_ss = duration_from_rate(dv_total_budget, dv_per_orbit_steady)

    # Print a concise summary
    print("\n=== ΔV Budget & Maintenance Duration ===")
    print(f"Isp: {Isp_sec:.0f} s, m0: {m0:.2f} kg, m_prop: {m_prop:.3f} kg")
    print(f"Total ΔV budget:               {dv_total_budget:.2f} m/s")
    print(
        f"Avg usage (0–{numOrbits:d} orbits): {dv_per_orbit_initial:.4f} m/s/orbit  "
        f"→ duration ≈ {orbits_init:.1f} orbits ({days_init:.1f} days)"
    )
    print(
        f"Steady-state usage (last 1/3): {dv_per_orbit_steady:.4f} m/s/orbit  "
        f"→ duration ≈ {orbits_ss:.1f} orbits ({days_ss:.1f} days)"
    )

    # Plot per-orbit usage estimate vs time (slope of moving window)
    win = max(5, int(0.1 * len(t_orb)))  # ~10% window
    mov_slope = np.empty_like(t_orb)
    mov_slope[:] = np.nan
    for k in range(win, len(t_orb)):
        m_loc, _ = np.polyfit(t_orb[k - win : k], dv_cum[k - win : k], 1)
        mov_slope[k] = m_loc

    plt.figure(7)
    plt.plot(t_orb, mov_slope, label="ΔV usage [m/s per orbit] (moving fit)")
    plt.axhline(dv_per_orbit_initial, linestyle="--", label="Avg (0–end)")
    plt.axhline(dv_per_orbit_steady, linestyle=":", label="Avg (steady-state)")
    plt.xlabel("time [orbit]")
    plt.ylabel("ΔV per orbit [m/s/orbit]")
    plt.title("Instantaneous ΔV Usage (moving linear fit)")
    plt.grid(True)
    plt.legend()

    # === Monthly ΔV estimates (general vs steady-state) ===
    # Use average Gregorian month length to avoid under/over-estimating.
    days_per_month = 365.2425 / 12.0  # ≈ 30.436875 days
    orbits_per_month = orbits_per_day * days_per_month

    #  overall average rate across the full run
    dv_month_general = dv_per_orbit_initial * orbits_per_month  # [m/s per month]

    # "Steady-state" = slope-based rate from the last third of the run
    dv_month_steady = dv_per_orbit_steady * orbits_per_month  # [m/s per month]

    print("\n=== Monthly ΔV Estimates ===")
    print(
        f"Orbits/day: {orbits_per_day:.2f}, Orbits/month: {orbits_per_month:.1f} (using {days_per_month:.3f} days/month)"
    )
    print(f"General monthly ΔV (overall avg): {dv_month_general:.2f} m/s per month")
    print(f"Steady-state monthly ΔV:          {dv_month_steady:.2f} m/s per month")

    # rolling monthly ΔV estimate over time ---
    # mov_slope is in [m/s per orbit]; convert to [m/s per month]
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

    # plt.show()
    plt.close("all")


if __name__ == "__main__":
    run(450)  # number of orbits

    """
    Total ΔV to maintain formation: 25.5066 m/s over 40 orbits
    Average ΔV per orbit:         0.6377 m/s/orbit
    Peak commanded force:         1.027296e-01 N

    === ΔV Budget & Maintenance Duration ===
    Isp: 2500 s, m0: 52.00 kg, m_prop: 1.200 kg
    Total ΔV budget:               572.40 m/s
    Avg usage (0-40 orbits): 0.6377 m/s/orbit  → duration ≈ 897.6 orbits (59.6 days)
    Steady-state usage (last 1/3): 0.0688 m/s/orbit  → duration ≈ 8314.9 orbits (552.3 days)
    """
