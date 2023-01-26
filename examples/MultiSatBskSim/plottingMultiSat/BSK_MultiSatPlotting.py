#
#  ISC License
#
#  Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
        figureList[pltName] = plt.figure(i+1)
    return figureList

# ------------------------------------- MAIN PLOT HANDLING ------------------------------------------------------ #
color_x = 'dodgerblue'
color_y = 'salmon'
color_z = 'lightgreen'
m2km = 1.0 / 1000.0

def plot_attitude(timeData, dataSigmaBN, id=None):
    """Plot the spacecraft attitude."""
    plt.figure(id)
    for idx in range(3):
        plt.plot(timeData, dataSigmaBN[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\sigma_' + str(idx+1) + '$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Attitude $\sigma_{B/N}$')
    return

def plot_attitude_error(timeData, dataSigmaBR, id=None):
    """Plot the spacecraft attitude error."""
    plt.figure(id)
    for idx in range(3):
        plt.plot(timeData, dataSigmaBR[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\sigma_' + str(idx+1) + '$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Attitude Tracking Error $\sigma_{B/R}$')
    return

def plot_attitude_reference(timeData, dataSigmaRN, id=None):
    """Plot the spacecraft attitude reference."""
    plt.figure(id)
    for idx in range(3):
        plt.plot(timeData, dataSigmaRN[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\sigma_' + str(idx+1) + '$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Attitude Reference $\sigma_{R/N}$')
    return

def plot_rate(timeData, dataOmegaBN, id=None):
    """Plot the body angular velocity rate tracking errors."""
    plt.figure(id)
    for idx in range(3):
        plt.plot(timeData, dataOmegaBN[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\omega_{BN,' + str(idx+1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Angular Rate (rad/s)')
    return

def plot_rate_error(timeData, dataOmegaBR, id=None):
    """Plot the body angular velocity rate tracking errors."""
    plt.figure(id)
    for idx in range(3):
        plt.plot(timeData, dataOmegaBR[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\omega_{BR,' + str(idx+1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Rate Tracking Error (rad/s)')
    return

def plot_rate_reference(timeData, dataOmegaRN, id=None):
    """Plot the body angular velocity rate reference."""
    plt.figure(id)
    for idx in range(3):
        plt.plot(timeData, dataOmegaRN[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\omega_{RN,' + str(idx+1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Reference Rate (rad/s)')
    return

def plot_rw_motor_torque(timeData, dataUsReq, dataRW, numRW, id=None):
    """Plot the RW motor torques."""
    plt.figure(id)
    for idx in range(numRW):
        plt.plot(timeData, dataUsReq[:, idx],
                 '--',
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label=r'$\hat u_{s,' + str(idx+1) + '}$')
        plt.plot(timeData, dataRW[idx],
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label='$u_{s,' + str(idx+1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Motor Torque (Nm)')
    return

def plot_rw_speeds(timeData, dataOmegaRW, numRW, id=None):
    """Plot the RW spin rates."""
    plt.figure(id)
    for idx in range(numRW):
        plt.plot(timeData, dataOmegaRW[:, idx] / macros.RPM,
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label=r'$\Omega_{' + str(idx+1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Speed (RPM) ')
    return

def plot_rw_voltages(timeData, dataVolt, numRW, id=None):
    """Plot the RW voltage inputs."""
    plt.figure(id)
    for idx in range(numRW):
        plt.plot(timeData, dataVolt[:, idx],
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label='$V_{' + str(idx+1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Voltage (V)')
    return

def plot_rw_power(timeData, dataRwPower, numRW, id=None):
    """Plot the RW power drain."""
    plt.figure(id)
    for idx in range(numRW):
        plt.plot(timeData, dataRwPower[idx],
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label='$p_{rw,' + str(idx+1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Power (W)')
    return

def plot_power(timeData, netData, supplyData, sinkData, id=None):
    """Plot the power inputs and outputs"""
    plt.figure(id)
    plt.plot(timeData, netData, label='Net Power')
    plt.plot(timeData, supplyData, label='Panel Power')
    plt.plot(timeData, sinkData, label='Power Draw')
    plt.xlabel('Time [min]')
    plt.ylabel('Power [W]')
    plt.grid(True)
    plt.legend(loc='lower right')
    return

def plot_battery(timeData, storageData, id=None):
    """Plot the energy inside the onboard battery"""
    plt.figure(id)
    plt.plot(timeData, storageData)
    plt.xlabel('Time [min]')
    plt.ylabel('Stored Energy [W-s]')
    return

def plot_rw_temperature(timeData, tempData, numRW, id=None):
    """Plot the reaction wheel temperatures"""
    plt.figure(id)
    for idx in range(numRW):
        plt.plot(timeData, tempData[idx],
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label='$T_{rw,' + str(idx+1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Temperatures [ºC]')
    return

def plot_thrust(timeData, thrustData, numThr, id=None):
    """Plot the thrusters net force output"""
    plt.figure(id)
    for idx in range(numThr):
        plt.plot(timeData, thrustData[idx],
                 color=unitTestSupport.getLineColor(idx, numThr),
                 label='$F_{thr,' + str(idx + 1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Thrust [N]')
    return

def plot_thrust_percentage(timeData, thrustData, numThr, id=None):
    """Plot the thrust as a percentage of maximum"""
    plt.figure(id)
    for idx in range(numThr):
        plt.plot(timeData, thrustData[idx],
                 color=unitTestSupport.getLineColor(idx, numThr),
                 label='$F_{thr,' + str(idx + 1) + '}$')
    plt.legend(loc='lower right')
    plt.ylim([0, 1.1])
    plt.xlabel('Time [min]')
    plt.ylabel('Thrust Percentage')
    return

def plot_fuel(timeData, fuelData, id=None):
    """Plot the fuel mass information"""
    plt.figure(id)
    plt.plot(timeData, fuelData)
    plt.xlabel('Time [min]')
    plt.ylabel('Stored Fuel Mass [kg]')
    return

def plot_orbit(r_BN, id=None):
    """Plot the spacecraft inertial orbitS."""
    plt.figure(id, figsize=(6, 5))
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

def plot_orbits(r_BN, numberSpacecraft, id=None):
    """Plot the spacecraft inertial orbits."""
    plt.figure(id, figsize=(6, 5))
    ax = plt.axes(projection='3d')
    for i in range(numberSpacecraft):
        ax.plot(r_BN[i][:, 0] * m2km, r_BN[i][:, 1] * m2km, r_BN[i][:, 2] * m2km,
                label="Spacecraft " + str(i),
                c=unitTestSupport.getLineColor(i, numberSpacecraft))
    ax.set_xlim3d(-8000, 8000)
    ax.set_ylim3d(-8000, 8000)
    ax.set_zlim3d(-8000, 8000)
    ax.scatter(0, 0, 0, c=color_x)
    ax.set_title('Spacecraft Inertial Orbits')
    ax.set_xlabel('x [km]')
    ax.set_ylabel('y [km]')
    ax.set_zlabel('z [km]')
    ax.legend(loc=2)
    return

def plot_relative_orbits(r_BN, numberSpacecraft, id=None):
    """Plot the spacecraft inertial orbits."""
    plt.figure(id, figsize=(6, 5))
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

def plot_orbital_element_differences(timeData, oed, id=None):
    """Plot the orbital element difference between the chief and another spacecraft."""
    plt.figure(id)
    plt.plot(timeData, oed[:, 0], label="da")
    plt.plot(timeData, oed[:, 1], label="de")
    plt.plot(timeData, oed[:, 2], label="di")
    plt.plot(timeData, oed[:, 3], label="dOmega")
    plt.plot(timeData, oed[:, 4], label="domega")
    plt.plot(timeData, oed[:, 5], label="dM")
    plt.legend()
    plt.xlabel("time [orbit]")
    plt.ylabel("Orbital Element Difference")

