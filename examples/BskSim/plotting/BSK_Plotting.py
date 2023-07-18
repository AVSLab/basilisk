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
import numpy as np
from Basilisk.utilities import RigidBodyKinematics
from Basilisk.utilities import macros as mc
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import unitTestSupport

# --------------------------------- COMPONENTS & SUBPLOT HANDLING ----------------------------------------------- #
color_x = 'dodgerblue'
color_y = 'salmon'
color_z = 'lightgreen'
m2km = 1.0 / 1000.0

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


def plot3components(timeAxis, vec, id=None):
    plt.figure(id)
    time = timeAxis * mc.NANO2MIN
    plt.xlabel('Time, min')
    plt.plot(time, vec[:, 0], color_x)
    plt.plot(time, vec[:, 1], color_y)
    plt.plot(time, vec[:, 2], color_z)


def plot_sigma(timeAxis, sigma, id=None):
    plot3components(timeAxis, sigma, id)
    plt.legend([r'$\sigma_1$', r'$\sigma_2$', r'$\sigma_3$'])
    plt.ylabel('MRP')


def plot_omega(timeAxis, omega, id=None):
    plot3components(timeAxis, omega, id)
    plt.ylabel('Angular Rate, rad/s')
    plt.legend([r'$\omega_1$', r'$\omega_2$', r'$\omega_3$'])

def subplot_sigma(subplot, timeAxis, sigma, id=None):
    plot3components(timeAxis, sigma, id)
    plt.legend([r'$\sigma_1$', r'$\sigma_2$', r'$\sigma_3$'])
    plt.ylabel('MRP')


def subplot_omega(subplot, timeAxis, omega, id=None):
    plot3components(timeAxis, omega, id)
    plt.ylabel('Angular Rate, rad/s')
    plt.legend([r'$\omega_1$', r'$\omega_2$', r'$\omega_3$'])


# ------------------------------------- MAIN PLOT HANDLING ------------------------------------------------------ #

# def plot_bodyTorque(Lb):
#     plt.figure()
#     plot3components(Lb)
#     plt.title('Body Torque $L_b$')
#     plt.ylabel('Body Torque, $N \cdot m$')
#     plt.legend(['$L_{b,1}$', '$L_{b,2}$', '$L_{b,3}$'])


def plot_controlTorque(timeAxis, Lr, id=None):
    plot3components(timeAxis, Lr, id)
    plt.ylabel(r'Control Torque, $N \cdot m$')
    plt.legend(['$L_{r,1}$', '$L_{r,2}$', '$L_{r,3}$'])
    plt.title('Control Torque $L_r$')
    return


def plot_trackingError(timeAxis, sigma_BR, omega_BR_B, id=None):
    # plt.figure(id)
    plt.subplot(211)
    plot_sigma(timeAxis, sigma_BR, id)
    plt.title(r'Att Error: $\sigma_{BR}$')

    plt.subplot(212)
    #plt.figure(id)
    plot_omega(timeAxis, omega_BR_B, id)
    plt.title(r'Rate Error: $^B{\omega_{BR}}$')
    return


def plot_attitudeGuidance(timeAxis, sigma_RN, omega_RN_N, id=None):
    plot_sigma(timeAxis, sigma_RN, id)
    plt.ylim([-1.0, 1.0])
    plt.title(r'Ref Att: $\sigma_{RN}$')

    plot_omega(timeAxis, omega_RN_N, id)
    plt.title(r'Ref Rate: $^N{\omega_{RN}}$')
    return


def plot_rotationalNav(timeAxis, sigma_BN, omega_BN_B, id=None):
    plt.figure()
    plot_sigma(timeAxis, sigma_BN, id)
    plt.title(r'Sc Att: $\sigma_{BN}$')

    plot_omega(timeAxis, omega_BN_B, id)
    plt.title(r'Sc Rate: $^B{\omega_{BN}}$')
    return


def plot_shadow_fraction(timeAxis, shadow_factor, id=None):
    plt.figure(id)
    plt.plot(timeAxis, shadow_factor)
    plt.xlabel('Time min')
    plt.ylabel('Shadow Fraction')
    return


def plot_sun_point(timeAxis, sunPoint, id=None):
    plot3components(timeAxis, sunPoint, id)
    plt.xlabel('Time')
    plt.ylabel('Sun Point Vec')
    return


def plot_orbit(r_BN, id=None):
    plt.figure(id)
    plt.xlabel('$R_x$, km')
    plt.ylabel('$R_y$, km')
    plt.plot(r_BN[:, 0] * m2km, r_BN[:, 1] * m2km, color_x)
    plt.scatter(0, 0, c=color_x)
    plt.title('Spacecraft Orbit')
    return


def plot_attitude_error(timeLineSet, dataSigmaBR, id=None):
    plt.figure(id)
    fig = plt.gcf()
    ax = fig.gca()
    vectorData = unitTestSupport.pullVectorSetFromData(dataSigmaBR)
    sNorm = np.array([np.linalg.norm(v) for v in vectorData])
    plt.plot(timeLineSet, sNorm,
             color=unitTestSupport.getLineColor(1, 3),
             )
    plt.xlabel('Time [min]')
    plt.ylabel(r'Attitude Error Norm $|\sigma_{B/R}|$')
    ax.set_yscale('log')


def plot_control_torque(timeLineSet, dataLr, id=None, livePlot=False):
    plt.figure(id)
    for idx in range(3):
        plt.plot(timeLineSet, dataLr[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$L_{r,' + str(idx) + '}$')
    if not livePlot:
        plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Control Torque $L_r$ [Nm]')


def plot_rate_error(timeLineSet, dataOmegaBR, id=None, livePlot=False):
    plt.figure(id)
    for idx in range(3):
        plt.plot(timeLineSet, dataOmegaBR[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\omega_{BR,' + str(idx) + '}$')
    if not livePlot:
        plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Rate Tracking Error [rad/s] ')
    return


def plot_orientation(timeLineSet, vectorPosData, vectorVelData, vectorMRPData, id=None, livePlot=False):
    data = np.empty([len(vectorPosData), 3])
    for idx in range(0, len(vectorPosData)):
        ir = vectorPosData[idx] / np.linalg.norm(vectorPosData[idx])
        hv = np.cross(vectorPosData[idx], vectorVelData[idx])
        ih = hv / np.linalg.norm(hv)
        itheta = np.cross(ih, ir)
        dcmBN = RigidBodyKinematics.MRP2C(vectorMRPData[idx])
        data[idx] = [np.dot(ir, dcmBN[0]), np.dot(itheta, dcmBN[1]), np.dot(ih, dcmBN[2])]
    plt.figure(id)
    labelStrings = (r'$\hat\imath_r\cdot \hat b_1$'
                    , r'${\hat\imath}_{\theta}\cdot \hat b_2$'
                    , r'$\hat\imath_h\cdot \hat b_3$')
    for idx in range(0, 3):
        plt.plot(timeLineSet, data[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=labelStrings[idx])
    if not livePlot:
        plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Orientation Illustration')


def plot_rw_cmd_torque(timeData, dataUsReq, numRW, id=None, livePlot=False):
    plt.figure(id)
    for idx in range(3):
        plt.plot(timeData, dataUsReq[:, idx],
                 '--',
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label=r'$\hat u_{s,' + str(idx) + '}$')
    if not livePlot:
        plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Motor Torque (Nm)')


def plot_rw_cmd_actual_torque(timeData, dataUsReq, dataRW, numRW, id=None, livePlot=False):
    """compare commanded and actual RW motor torques"""
    plt.figure(id)
    for idx in range(numRW):
        plt.plot(timeData, dataUsReq[:, idx],
                 '--',
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label=r'$\hat u_{s,' + str(idx) + '}$')
        plt.plot(timeData, dataRW[idx],
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label='$u_{s,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Motor Torque (Nm)')


def plot_rw_speeds(timeData, dataOmegaRW, numRW, id=None, livePlot=False):
    plt.figure(id)
    for idx in range(numRW):
        plt.plot(timeData, dataOmegaRW[:, idx] / mc.RPM,
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label=r'$\Omega_{' + str(idx) + '}$')
    if not livePlot:
        plt.legend(loc='upper right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Speed (RPM) ')

def plot_rw_friction(timeData, dataFrictionRW, numRW, dataFaultLog=[],  id=None, livePlot=False):
    plt.figure(id)
    for idx in range(numRW):
        plt.plot(timeData, dataFrictionRW[idx],
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label=r'$RW_{' + str(idx+1) + '} Friction$')
    if dataFaultLog:
        # fourth column of dataFaultLog is the fault times
        plt.scatter([row[3] for row in dataFaultLog], np.zeros(len(dataFaultLog)), marker="x", color=(1,0,0),
            label='Faults')
    if not livePlot:
        plt.legend(loc='upper right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Static Friction ')


def plot_planet(oe, planet):
    b = oe.a * np.sqrt(1 - oe.e * oe.e)
    plt.figure(figsize=np.array((1.0, b / oe.a)) * 4.75, dpi=100)
    plt.axis(np.array([-oe.a, oe.a, -b, b]) / 1000 * 1.75)
    # draw the planet
    fig = plt.gcf()
    ax = fig.gca()
    planetColor = '#008800'
    planetRadius = planet.radEquator / 1000
    ax.add_artist(plt.Circle((0, 0), planetRadius, color=planetColor))
    plt.xlabel('$i_e$ Cord. [km]')
    plt.ylabel('$i_p$ Cord. [km]')
    plt.grid()


def plot_peri_and_orbit(oe, mu, r_BN_N, v_BN_N, id=None):
    # draw the actual orbit
    rData = []
    fData = []
    p = oe.a * (1 - oe.e * oe.e)
    for idx in range(0, len(r_BN_N)):
        oeData = orbitalMotion.rv2elem(mu, r_BN_N[idx], v_BN_N[idx])
        rData.append(oeData.rmag)
        fData.append(oeData.f + oeData.omega - oe.omega)
    plt.figure(id)
    plt.plot(rData * np.cos(fData) / 1000, rData * np.sin(fData) / 1000, color='#aa0000', linewidth=3.0)
    # draw the full osculating orbit from the initial conditions
    fData = np.linspace(0, 2 * np.pi, 100)
    rData = []
    for idx in range(0, len(fData)):
        rData.append(p / (1 + oe.e * np.cos(fData[idx])))
    plt.plot(rData * np.cos(fData) / 1000, rData * np.sin(fData) / 1000, '--', color='#555555')


def plot_rel_orbit(timeData, r_chief, r_deputy, id=None, livePlot=False):
    plt.figure(id)
    x = np.array(r_chief[:, 0]) - np.array(r_deputy[:, 0])
    y = np.array(r_chief[:, 1]) - np.array(r_deputy[:, 1])
    z = np.array(r_chief[:, 2]) - np.array(r_deputy[:, 2])
    plt.plot(timeData, x, label="x")
    plt.plot(timeData, y, label="y")
    plt.plot(timeData, z, label="z")
    if not livePlot:
        plt.legend()
    plt.grid()


def plot_position(time, r_BN_N_truth, r_BN_N_meas, tTN, r_TN_N, id=None):
    """Plot the position result."""
    fig, ax = plt.subplots(3, sharex=True, num=id)
    fig.add_subplot(111, frameon=False)
    plt.tick_params(labelcolor='none', top=False, bottom=False, left=False, right=False)

    ax[0].plot(time, r_BN_N_meas[:, 0], 'k*', label='measurement', markersize=2)
    ax[1].plot(time, r_BN_N_meas[:, 1], 'k*', markersize=2)
    ax[2].plot(time, r_BN_N_meas[:, 2], 'k*', markersize=2)

    ax[0].plot(time, r_BN_N_truth[:, 0], label='truth')
    ax[1].plot(time, r_BN_N_truth[:, 1])
    ax[2].plot(time, r_BN_N_truth[:, 2])

    ax[0].plot(tTN, r_TN_N[0], 'rx', label='target')
    ax[1].plot(tTN, r_TN_N[1], 'rx')
    ax[2].plot(tTN, r_TN_N[2], 'rx')

    plt.xlabel('Time [min]')
    plt.title('Spacecraft Position')

    ax[0].set_ylabel('${}^Nr_{BN_1}$ [m]')
    ax[1].set_ylabel('${}^Nr_{BN_2}$ [m]')
    ax[2].set_ylabel('${}^Nr_{BN_3}$ [m]')

    ax[0].legend(loc='upper right')


def plot_velocity(time, v_BN_N_truth, v_BN_N_meas, id=None):
    """Plot the velocity result."""
    fig, ax = plt.subplots(3, sharex=True, num=id)
    fig.add_subplot(111, frameon=False)
    plt.tick_params(labelcolor='none', top=False, bottom=False, left=False, right=False)

    ax[0].plot(time, v_BN_N_meas[:, 0], 'k*', label='measurement', markersize=2)
    ax[1].plot(time, v_BN_N_meas[:, 1], 'k*', markersize=2)
    ax[2].plot(time, v_BN_N_meas[:, 2], 'k*', markersize=2)

    ax[0].plot(time, v_BN_N_truth[:, 0], label='truth')
    ax[1].plot(time, v_BN_N_truth[:, 1])
    ax[2].plot(time, v_BN_N_truth[:, 2])

    plt.xlabel('Time [min]')
    plt.title('Spacecraft Velocity')

    ax[0].set_ylabel('${}^Nv_{BN_1}$ [m/s]')
    ax[1].set_ylabel('${}^Nv_{BN_2}$ [m/s]')
    ax[2].set_ylabel('${}^Nv_{BN_3}$ [m/s]')

    ax[0].legend()


def plot_surface_rel_velocity(timeData, r_BN_N, v_BN_N, sigma_PN, omega_PN_P, id=None):
    v_BS_S = np.empty([len(r_BN_N), 3])
    for idx in range(0, len(r_BN_N)):
        dcmPN = RigidBodyKinematics.MRP2C(sigma_PN[idx])
        omega_PN_N = np.dot(dcmPN.transpose(), omega_PN_P[idx])
        s1Hat_N = np.cross(omega_PN_N, r_BN_N[idx])/np.linalg.norm(np.cross(omega_PN_N, r_BN_N[idx]))
        s3Hat_N = r_BN_N[idx]/np.linalg.norm(r_BN_N[idx])
        s2Hat_N = np.cross(s3Hat_N, s1Hat_N)/np.linalg.norm(np.cross(s3Hat_N, s1Hat_N))
        dcmSN = np.array([s1Hat_N.transpose(), s2Hat_N.transpose(), s3Hat_N.transpose()])
        v_BS_S[idx] = np.dot(dcmSN, v_BN_N[idx] - np.cross(omega_PN_N, r_BN_N[idx]))

    fig, ax = plt.subplots(3, sharex=True, num=id)
    fig.add_subplot(111, frameon=False)
    plt.tick_params(labelcolor='none', top=False, bottom=False, left=False, right=False)

    ax[0].plot(timeData, v_BS_S[:, 0])
    ax[1].plot(timeData, v_BS_S[:, 1])
    ax[2].plot(timeData, v_BS_S[:, 2])

    plt.xlabel('Time [min]')
    plt.title('Surface Relative Velocity')

    ax[0].set_ylabel('${}^Sv_{BS_1}$ [m/s]')
    ax[1].set_ylabel('${}^Sv_{BS_2}$ [m/s]')
    ax[2].set_ylabel('${}^Sv_{BS_3}$ [m/s]')
