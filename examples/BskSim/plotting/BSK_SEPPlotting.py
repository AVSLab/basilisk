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
import matplotlib.pyplot as plt
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from Basilisk.utilities import macros
from Basilisk.utilities import unitTestSupport


# --------------------------------- COMPONENTS & SUBPLOT HANDLING ----------------------------------------------- #

def show_all_plots():
    plt.show()


def clear_all_plots():
    plt.close("all")


def save_all_plots(fileName, figureNames):
    figureList = {}
    numFigures = len(figureNames)
    for i in range(0, numFigures):
        pltName = fileName + "_" + figureNames[i]
        figureList[pltName] = plt.figure(i + 1)
    return figureList


# ------------------------------------- MAIN PLOT HANDLING ------------------------------------------------------ #
color_x = 'dodgerblue'
color_y = 'salmon'
color_z = 'lightgreen'
m2km = 1.0 / 1000.0


def plot_attitude(timeData, dataSigmaBN, dataSigmaRN, figID=None):
    """Plot the spacecraft attitude."""
    plt.figure(figID)
    for idx in range(3):
        plt.plot(timeData, dataSigmaBN[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\sigma_{BN,' + str(idx + 1) + '}$')
    for idx in range(3):
        plt.plot(timeData, dataSigmaRN[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3), linestyle='dashed',
                 label=r'$\sigma_{RN,' + str(idx + 1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Attitude $\sigma$')
    return


def plot_attitude_error(timeData, dataSigmaBR, figID=None):
    """Plot the spacecraft attitude error."""
    plt.figure(figID)
    for idx in range(3):
        plt.plot(timeData, dataSigmaBR[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\sigma_{BR,' + str(idx + 1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Attitude Tracking Error $\sigma_{B/R}$')
    return


def plot_attitude_reference(timeData, dataSigmaRN, figID=None):
    """Plot the spacecraft attitude reference."""
    plt.figure(figID)
    for idx in range(3):
        plt.plot(timeData, dataSigmaRN[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\sigma_' + str(idx + 1) + '$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Attitude Reference $\sigma_{R/N}$')
    return


def plot_rate(timeData, dataOmegaBN, dataOmegaRN, figID=None):
    """Plot the body angular velocity rate tracking errors."""
    plt.figure(figID)
    for idx in range(3):
        plt.plot(timeData, dataOmegaBN[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\omega_{BN,' + str(idx + 1) + '}$')
    for idx in range(3):
        plt.plot(timeData, dataOmegaRN[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3), linestyle='dashed',
                 label=r'$\omega_{RN,' + str(idx + 1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Angular Rate (rad/s)')
    return


def plot_rate_error(timeData, dataOmegaBR, figID=None):
    """Plot the body angular velocity rate tracking errors."""
    plt.figure(figID)
    for idx in range(3):
        plt.plot(timeData, dataOmegaBR[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\omega_{BR,' + str(idx + 1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Rate Tracking Error (rad/s)')
    return


def plot_rate_reference(timeData, dataOmegaRN, figID=None):
    """Plot the body angular velocity rate reference."""
    plt.figure(figID)
    for idx in range(3):
        plt.plot(timeData, dataOmegaRN[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\omega_{RN,' + str(idx + 1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Reference Rate (rad/s)')
    return


def plot_rw_motor_torque(timeData, dataUsReq, dataRW, numRW, figID=None):
    """Plot the RW motor torques."""
    plt.figure(figID)
    for idx in range(numRW):
        plt.plot(timeData, dataUsReq[:, idx],
                 '--',
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label=r'$\hat u_{s,' + str(idx + 1) + '}$')
        plt.plot(timeData, dataRW[idx],
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label='$u_{s,' + str(idx + 1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Motor Torque (Nm)')
    return


def plot_rw_speeds(timeData, dataOmegaRW, numRW, OmegaMax, figID=None):
    """Plot the RW spin rates."""
    dataOmegaMax = OmegaMax * np.ones(len(timeData))
    plt.figure(figID)
    for idx in range(numRW):
        plt.plot(timeData, dataOmegaRW[:, idx] / macros.RPM,
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label=r'$\Omega_{' + str(idx + 1) + '}$')
    plt.plot(timeData, 0.75*dataOmegaMax / macros.RPM, color='black', 
             linestyle='dashed', label=r'$75\% \Omega_{max}$')
    plt.plot(timeData, -0.75*dataOmegaMax / macros.RPM, color='black', 
             linestyle='dashed')
    plt.plot(timeData, dataOmegaMax / macros.RPM, color='black', 
             linestyle='solid', label=r'$\Omega_{max}$')
    plt.plot(timeData, -dataOmegaMax / macros.RPM, color='black', 
             linestyle='solid')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Speed (RPM) ')
    return


def plot_rw_voltages(timeData, dataVolt, numRW, figID=None):
    """Plot the RW voltage inputs."""
    plt.figure(figID)
    for idx in range(numRW):
        plt.plot(timeData, dataVolt[:, idx],
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label='$V_{' + str(idx + 1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Voltage (V)')
    return


def plot_rw_power(timeData, dataRwPower, numRW, figID=None):
    """Plot the RW power drain."""
    plt.figure(figID)
    for idx in range(numRW):
        plt.plot(timeData, dataRwPower[idx],
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label='$p_{rw,' + str(idx + 1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Power (W)')
    return


def plot_power(timeData, netData, supplyData, sinkData, figID=None):
    """Plot the power inputs and outputs"""
    plt.figure(figID)
    plt.plot(timeData, netData, label='Net Power')
    plt.plot(timeData, supplyData, label='Panel Power')
    plt.plot(timeData, sinkData, label='Power Draw')
    plt.xlabel('Time [min]')
    plt.ylabel('Power [W]')
    plt.grid(True)
    plt.legend(loc='lower right')
    return


def plot_battery(timeData, storageData, figID=None):
    """Plot the energy inside the onboard battery"""
    plt.figure(figID)
    plt.plot(timeData, storageData)
    plt.xlabel('Time [min]')
    plt.ylabel('Stored Energy [W-s]')
    return


def plot_rw_temperature(timeData, tempData, numRW, figID=None):
    """Plot the reaction wheel temperatures"""
    plt.figure(figID)
    for idx in range(numRW):
        plt.plot(timeData, tempData[idx],
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label='$T_{rw,' + str(idx + 1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Temperatures [ºC]')
    return


def plot_thrust(timeData, thrustData, numThr, figID=None):
    """Plot the thrusters net force output"""
    plt.figure(figID)
    for idx in range(numThr):
        plt.plot(timeData, thrustData[idx],
                 color=unitTestSupport.getLineColor(idx, numThr),
                 label='$F_{thr,' + str(idx + 1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Thrust [N]')
    return


def plot_thrust_percentage(timeData, thrustData, numThr, figID=None):
    """Plot the thrust as a percentage of maximum"""
    plt.figure(figID)
    for idx in range(numThr):
        plt.plot(timeData, thrustData[idx],
                 color=unitTestSupport.getLineColor(idx, numThr),
                 label='$F_{thr,' + str(idx + 1) + '}$')
    plt.legend(loc='lower right')
    plt.ylim([0, 1.1])
    plt.xlabel('Time [min]')
    plt.ylabel('Thrust Percentage')
    return


def plot_fuel(timeData, fuelData, figID=None):
    """Plot the fuel mass information"""
    plt.figure(figID)
    plt.plot(timeData, fuelData)
    plt.xlabel('Time [min]')
    plt.ylabel('Stored Fuel Mass [kg]')
    return


def plot_orbit(r_BN, figID=None):
    """Plot the spacecraft inertial orbitS."""
    plt.figure(figID, figsize=(6, 5))
    ax = plt.axes(projection='3d')
    ax.plot(r_BN[:, 0] * m2km, r_BN[:, 1] * m2km, r_BN[:, 2] * m2km,
            label="Spacecraft")
    ax.set_xlim3d(-8000, 8000)
    ax.set_ylim3d(-8000, 8000)
    ax.set_zlim3d(-8000, 8000)
    ax.scatter(0, 0, 0, c=color_x)
    ax.set_title('Spacecraft Inertial Orbit')
    ax.set_xlabel('x [km]')
    ax.set_ylabel('y [km]')
    ax.set_zlabel('z [km]')
    ax.legend(loc=2)
    return


def plot_orbits(r_BN, numberSpacecraft, figID=None):
    """Plot the spacecraft inertial orbits."""
    fig = plt.figure(figID, figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_box_aspect((16000, 16000, 16000))
    # plt.ticklabel_format(style='sci', axis='both', scilimits=(0, 0))

    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)

    # r = 6378.1
    r = 2440.5
    x = r * np.outer(np.cos(u), np.sin(v))
    y = r * np.outer(np.sin(u), np.sin(v))
    z = r * np.outer(np.ones(np.size(u)), np.cos(v))

    ax.plot_surface(x, y, z, rstride=4, cstride=4, color='r', linewidth=0, alpha=0.5)
    for i in range(numberSpacecraft):
        ax.plot(r_BN[i][:, 0] * m2km, r_BN[i][:, 1] * m2km, r_BN[i][:, 2] * m2km,
                label="Spacecraft " + str(i),
                c=unitTestSupport.getLineColor(i, numberSpacecraft)
                )
    ax.scatter(0, 0, 0, c=color_x)

    ax.set_title('Spacecraft Inertial Orbits', fontsize=20)
    ax.set_xlabel('x [km]', fontsize=16)
    ax.set_ylabel('y [km]', fontsize=16)
    ax.set_zlabel('z [km]', fontsize=16)
    ax.legend(loc=2, bbox_to_anchor=(0.1, 0.95), prop={"size": 14})
    ax.tick_params(axis='both', which='major', labelsize=10)
    ax.set_xticks([-6000, -3000, 0, 3000, 6000])
    ax.set_yticks([-6000, -3000, 0, 3000, 6000])
    ax.set_zticks([-6000, -3000, 0, 3000, 6000])

    ax.set_xlim3d(-8000, 8000)
    ax.set_ylim3d(-8000, 8000)
    ax.set_zlim3d(-8000, 8000)
    return


def plot_relative_orbits(r_BN, numberSpacecraft, figID=None):
    """Plot the spacecraft inertial orbits."""
    plt.figure(figID, figsize=(6, 5))
    ax = plt.axes(projection='3d')
    for i in range(numberSpacecraft):
        ax.plot(r_BN[i][:, 0] * m2km, r_BN[i][:, 1] * m2km, r_BN[i][:, 2] * m2km,
                label="Spacecraft " + str(i),
                c=unitTestSupport.getLineColor(i, numberSpacecraft))
    ax.set_box_aspect((np.ptp(r_BN[i][:, 0]), np.ptp(r_BN[i][:, 1]), np.ptp(r_BN[i][:, 2])))
    ax.scatter(0, 0, 0, c=color_x)
    ax.set_title('Spacecraft Relative Orbits in Hill Frame')
    ax.set_xlabel('$i_r$ [km]')
    ax.set_ylabel(r'$i_{\theta}$ [km]')
    ax.set_zlabel('$i_h$ [km]')
    ax.legend(loc=2)
    return


def plot_orbital_element_differences(timeData, oed, figID=None):
    """Plot the orbital element difference between the chief and another spacecraft."""
    plt.figure(figID)
    plt.plot(timeData, oed[:, 0], label="da")
    plt.plot(timeData, oed[:, 1], label="de")
    plt.plot(timeData, oed[:, 2], label="di")
    plt.plot(timeData, oed[:, 3], label="dOmega")
    plt.plot(timeData, oed[:, 4], label="domega")
    plt.plot(timeData, oed[:, 5], label="dM")
    plt.legend()
    plt.xlabel("time [orbit]")
    plt.ylabel("Orbital Element Difference")


def plot_earth_sun_boresight(timeData, dataEarthBoreAngle, dataSensitiveBoreAngle, dataSunBoreAngles, figID=None):
    """Plot the boresight pointing error."""
    plt.figure(figID)
    plt.plot(timeData, dataEarthBoreAngle / np.pi * 180, color='tab:green', label=r'$+y_{hub} - Earth$')
    plt.plot(timeData, dataSensitiveBoreAngle / np.pi * 180, color='tab:red', label=r'$-y_{hub} - Sun$')
    for sunAngle in dataSunBoreAngles:
        plt.plot(timeData, sunAngle / np.pi * 180, color='tab:orange', linestyle='dashed', label=r'$y_{SA}-Sun$')
    plt.legend(loc='best')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Boresight angle $\Delta \theta$ [deg]')
    # plt.title(r'Pointing Performance')


def plot_inertial_sun_boresight(timeData, dataInertialBoreAngle, dataSensitiveBoreAngle, dataSunBoreAngles, figID=None):
    """Plot the boresight pointing error."""
    plt.figure(figID)
    plt.plot(timeData, dataInertialBoreAngle / np.pi * 180, color='tab:blue', label=r'Thrust - Inertial')
    plt.plot(timeData, dataSensitiveBoreAngle / np.pi * 180, color='tab:red', label=r'$-y_{hub} - Sun$')
    for sunAngle in dataSunBoreAngles:
        plt.plot(timeData, sunAngle / np.pi * 180, color='tab:orange', linestyle='dashed', label=r'$y_{SA}-Sun$')
    plt.legend(loc='best')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Boresight angle $\Delta \theta$ [deg]')
    # plt.title(r'Pointing Performance')


def plot_thruster_offset(timeData, dataThrusterOffset, figID=None):
    """Plot the boresight pointing error."""
    plt.figure(figID)
    plt.plot(timeData, dataThrusterOffset / np.pi * 180, color='tab:blue', label=r'$\Delta \theta$')
    plt.legend(loc='best')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Offset thruster / CM [deg]')
    # plt.title(r'Pointing Performance')


def plot_solar_array_angle(timeData, dataAngle, dataRefAngle, figID=None):
    """Plot the solar array angles."""
    plt.figure(figID)
    for i, angle in enumerate(dataAngle):
        plt.plot(timeData, angle / np.pi * 180, color='C'+str(i), label=r'$\alpha_' + str(i+1) + '$')
    for i, angle in enumerate(dataRefAngle):
        plt.plot(timeData, angle / np.pi * 180, color='C'+str(i), linestyle='dashed', label=r'$\alpha_{R,' + str(i+1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Solar Array Angles [deg]')


def plot_solar_array_angle_rate(timeData, dataAngleRate, figID=None):
    """Plot the solar array angles."""
    plt.figure(figID)
    for i, angleRate in enumerate(dataAngleRate):
        plt.plot(timeData, angleRate / np.pi * 180, label=r'$\dot{\alpha}_' + str(i+1) + '$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Solar Array Angle Rates [deg/s]')


def plot_solar_array_torques(timeData, dataArrayTorques, figID=None):
    """Plot the solar array torques."""
    plt.figure(figID)
    for i, torque in enumerate(dataArrayTorques):
        plt.plot(timeData, torque[:, 0], label=r'$u_' + str(i+1) + '$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Solar Array Torques [Nm]')


def plot_platform_angle(timeData, dataAngle, dataRefAngle, figID=None):
    """Plot the solar array angles."""
    plt.figure(figID)
    for i, angle in enumerate(dataAngle):
        plt.plot(timeData, angle / np.pi * 180, color='C'+str(i), label=r'$\theta_' + str(i+1) + '$')
    for i, angle in enumerate(dataRefAngle):
        plt.plot(timeData, angle / np.pi * 180, color='C'+str(i), linestyle='dashed', label=r'$\theta_{R,' + str(i+1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Platform Angles [deg]')


def plot_platform_torques(timeData, dataTorques, dataSwitch, dataReqTorque, index, figID=None):
    """Plot the solar array torques."""
    plt.figure(figID)
    for i in range(index):
        torque = []
        for j in range(len(dataTorques)):
            if dataSwitch[j][i] == 1:
                torque.append(0)
            else:
                torque.append(dataTorques[j][i])
        plt.plot(timeData, torque, color='C'+str(i), label=r'$u_' + str(i+1) + '$')
    for i, torque in enumerate(dataReqTorque):
        plt.plot(timeData, torque[:, 0], color='C'+str(i), linestyle='dashed', label=r'$u_{R,' + str(i+1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Platform Torques [Nm]')


def plot_external_torque(timeData, dataTorques, torqueString, figID=None):
    """Plot the external torques."""
    plt.figure(figID)
    for idx in range(3):
        plt.plot(timeData, dataTorques[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'${}^BL_' + str(idx+1) + '$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(torqueString + r' [Nm]')

def plot_inertial_pointing(timeData, dataInertialBoreAngle, epsilon, figID=None):
    """Plot the boresight pointing error."""
    dataBounds = epsilon * np.ones(len(timeData))
    plt.figure(figID)
    plt.plot(timeData, dataInertialBoreAngle / np.pi * 180, color='tab:blue', label=r'Thrust pointing error')
    plt.plot(timeData, dataBounds, color='tab:blue', linestyle='dashed', label=r'+$\epsilon$')
    plt.plot(timeData, -dataBounds, color='tab:blue', linestyle='dashed', label=r'-$\epsilon$')
    plt.ylim(-epsilon * 1.25, epsilon * 1.25)
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Thr. Pointing Accuracy [deg]')