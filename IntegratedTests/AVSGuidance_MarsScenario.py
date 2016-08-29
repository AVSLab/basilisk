'''
Copyright (c) 2016, Autonomous Vehicle Systems Lab, Univeristy of Colorado at Boulder

Permission to use, copy, modify, and/or distribute this software for any
purpose with or without fee is hereby granted, provided that the above
copyright notice and this permission notice appear in all copies.

THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

'''
import sys, os, inspect

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
sys.path.append(path + '/../PythonModules/')
import AVSSim
import matplotlib.pyplot as plt
#plt.rcParams.bbox_inches="tight"
from mpl_toolkits.mplot3d import Axes3D
import ctypes
import math
import MessagingAccess
import SimulationBaseClass
import sim_model
import logging
import numpy as np
from numpy import cos, sin
from numpy import linalg as la
import macros as mc
import astroFunctions as af
import RigidBodyKinematics as rbk


# ------------------- PLOTS DIRECTORY ------------------- #
paperPath = '/Users/marcolsmargenet/Desktop/AIAApaper_new/Figures/'
arePlotsSaved = False
if arePlotsSaved:
    plt.rcParams['figure.figsize'] = 1.75, 1.5
    plt.rcParams.update({'font.size': 6})

color_x = 'dodgerblue'
color_y = 'limegreen'
color_z = 'r'

def plotLabels_sigma():
    plt.legend(['$\sigma_1$', '$\sigma_2$', '$\sigma_3$'])
    plt.xlabel('Time, h')
    plt.ylabel('MRP Attitude Set')
    #plt.ylim([-1.0, 1.0])
def plotLabels_omega():
    plt.legend(['$\omega_1$', '$\omega_2$', '$\omega_3$'])
    plt.xlabel('Time, h')
    plt.ylabel('Angular Rate, rad/s')
# ------------------- PLOTTING FUNCTIONS ------------------- #

def plotRV_mag(r_BN_N, v_BN_N):
    t = r_BN_N[:, 0] * mc.NANO2SEC
    print 'r_BN_0 = ', r_BN_N[0, 1:]
    print 'v_BN_0 = ', v_BN_N[0, 1:]
    r = np.array([])
    v = np.array([])
    for i in range(len(t)):
        r = np.append(r, la.norm(r_BN_N[i, 1:]))
        v = np.append(v, la.norm(v_BN_N[i, 1:]))
    print 'R_mag = ', r
    print 'V_mag = ', v
    print 'Initial Pos Vec: ', r_BN_N[0,:]
    print 'Final Pos Vec: ', r_BN_N[len(t)-1,:]
    print '\n'

    plt.figure(0)
    plt.plot(t, r * 1E-3, 'b')
    plt.title('Pos Mag [km]')

    plt.figure(1)
    plt.plot(t, v * 1E-3, 'g')
    plt.title('Vel Mag [km]')
    plt.figure(2)
    plt.plot(t, r, 'b', t, v, 'g')
    plt.legend(['Pos Mag [km]', 'Vel Mag [km/s]'])

def plotRotNav(sigma_BN, omega_BN_B):
    print 'sigma_BN = ', sigma_BN[:, 1:]
    print 'omega_BN_N = ', omega_BN_B[:, 1:]
    print '\n'
    t = sigma_BN[:, 0] * mc.NANO2SEC / 3600.0

    plt.figure(5)
    plt.ylim([-1.0, 1.0])
    plt.plot(t, sigma_BN[:, 1] , t, sigma_BN[:, 2], t, sigma_BN[:, 3])
    plotLabels_sigma()
    plt.title(TheAVSSim.modeRequest + ': $\sigma_{BN}$')

    plt.figure(6)
    plt.plot(t, omega_BN_B[:, 1], t, omega_BN_B[:, 2], t, omega_BN_B[:, 3])
    plt.legend(['$x_1$', '$x_2$', '$x_3$'])
    plt.title(TheAVSSim.modeRequest + ': $\omega_{BN, B}$')

def plotReference(sigma_RN, omega_RN_N, domega_R0N_N):
    print 'sigma_RN = ', sigma_RN[:, 1:]
    print 'omega_RN_N = ', omega_RN_N[:, 1:]
    print '\n'
    t = sigma_RN[:, 0] * mc.NANO2SEC / 3600.0

    plt.figure(10)
    plt.plot(t, sigma_RN[:, 1], color_x)
    plt.plot(t, sigma_RN[:, 2], color_y)
    plt.plot(t, sigma_RN[:, 3], color_z)
    plotLabels_sigma()
    if arePlotsSaved:
        plt.savefig(paperPath + TheAVSSim.modeRequest + "/sigma_RN.pdf", bbox_inches='tight')
    else:
        plt.title(TheAVSSim.modeRequest + ': Reference Att $\sigma_{RN}$')

    plt.figure(11)
    plt.plot(t, omega_RN_N[:, 1], color_x)
    plt.plot(t, omega_RN_N[:, 2], color_y)
    plt.plot(t, omega_RN_N[:, 3], color_z)
    plotLabels_omega()
    if arePlotsSaved:
        plt.savefig(paperPath + TheAVSSim.modeRequest + "/omega_RN_N.pdf", bbox_inches='tight')
    else:
        plt.title(TheAVSSim.modeRequest + ' : Reference Rate $\omega_{RN}$')

    plt.figure(12)
    plt.plot(t, domega_R0N_N[:, 1], color_x)
    plt.plot(t, domega_R0N_N[:, 2], color_y)
    plt.plot(t, domega_R0N_N[:, 3], color_z)
    plt.legend(['$\dot{\omega_1}$', '$\dot{\omega}_2$', '$\dot{\omega}_3$'])
    plt.xlabel('Time, h')
    plt.ylabel('Angular Acceleration, rad/s$^2$')
    plt.title(TheAVSSim.modeRequest + ' : Reference Acceleration $\dot{\omega}_{RN}$')

def plotBaseReference(sigma_R0N, omega_R0N_N):
    print 'sigma_R0N = ', sigma_R0N[:, 1:]
    print 'omega_R0N_N = ', omega_R0N_N[:, 1:]
    print '\n'
    t = sigma_R0N[:, 0] * mc.NANO2SEC / 3600.0

    plt.figure(15)
    plt.plot(t, sigma_R0N[:, 1], color_x)
    plt.plot(t, sigma_R0N[:, 2], color_y)
    plt.plot(t, sigma_R0N[:, 3], color_z)
    plotLabels_sigma()
    if arePlotsSaved:
        plt.savefig(paperPath + TheAVSSim.modeRequest + "/sigma_R0N.pdf", bbox_inches='tight')
    else:
        plt.title(': Base Reference Att $\sigma_{R0N}$')

    plt.figure(16)
    plt.plot(t, omega_R0N_N[:, 1] * 1E3, color_x)
    plt.plot(t, omega_R0N_N[:, 2] * 1E3, color_y)
    plt.plot(t, omega_R0N_N[:, 3] * 1E3, color_z)
    plotLabels_omega()
    plt.ylabel('Angular Rate, $10^3$ rad/s')
    if arePlotsSaved:
        plt.savefig(paperPath + TheAVSSim.modeRequest + "/omega_R0N_N.pdf", bbox_inches='tight')
    else:
        plt.title(TheAVSSim.modeRequest +': Base Reference Rate $\omega_{R0/N}$')

def plotEulerRates(eulerRates):
    print 'eulerRates = ', eulerRates[:, 1:]
    print '\n'
    t_vec = eulerRates[:, 0] * mc.NANO2SEC / 3600.0
    psiDot_vec = eulerRates[:, 1]
    thetaDot_vec = eulerRates[:, 2]
    phiDot_vec = eulerRates[:, 3]
    plt.figure(20)
    plt.plot(t_vec, psiDot_vec, color_x)
    plt.plot(t_vec, thetaDot_vec, color_y)
    plt.plot(t_vec, phiDot_vec, color_z)
    plt.legend(['$\dot\psi$', '$\dot\Theta$', '$\dot\phi$'])
    plt.xlabel('Time, h')
    plt.ylabel('3-2-1 Euler Rates, rad/s')
    if arePlotsSaved:
        plt.savefig(paperPath+TheAVSSim.modeRequest+"/euler_rates.pdf", bbox_inches='tight')
    else:
        plt.title(TheAVSSim.modeRequest + ': 3-2-1 Euler Rates wrt Base Ref')


def plotEulerSet(eulerSet):
    print 'eulerSet = ', eulerSet[:, 1:]
    print '\n'
    t_vec = eulerSet[:, 0] * mc.NANO2SEC / 3600.0
    psi_vec = eulerSet[:, 1]
    theta_vec = eulerSet[:, 2]
    phi_vec = eulerSet[:, 3]

    rx_vec = np.array([])
    ry_vec = np.array([])
    rz_vec = np.array([])
    for i in range(len(t_vec)):
        rx = cos(theta_vec[i]) * cos(psi_vec[i])
        ry = cos(theta_vec[i]) * sin(psi_vec[i])
        rz = sin(theta_vec[i])
        rx_vec = np.append(rx_vec, rx)
        ry_vec = np.append(ry_vec, ry)
        rz_vec = np.append(rz_vec, rz)

    def plot321Angles():
        plt.figure(21)
        plt.plot(t_vec, psi_vec, color_x)
        plt.plot(t_vec, theta_vec, color_y)
        plt.plot(t_vec, phi_vec, color_z)
        plt.legend(['$\psi$', '$\Theta$', '$\phi$'])
        plt.xlabel('Time, h')
        plt.ylabel('3-2-1 Euler Set, rad')
        if arePlotsSaved:
            plt.savefig(paperPath + TheAVSSim.modeRequest + "/euler_set.pdf", bbox_inches='tight')
        else:
            plt.title(TheAVSSim.modeRequest + ': 3-2-1 Euler Set wrt Base Ref')

    def plot_boresight3D():
        fig = plt.figure(24, figsize=(5,4))
        ax = fig.add_subplot(111, projection='3d')
        #ax.view_init(elev=0., azim=0.)
        ax.plot(rx_vec, ry_vec, rz_vec)
        max_range = np.array([rx_vec.max() - rx_vec.min(), ry_vec.max() - ry_vec.min(), rz_vec.max() - rz_vec.min()]).max()
        Xb = 0.5 * max_range * np.mgrid[-1:2:2, -1:2:2, -1:2:2][0].flatten() + 0.5 * (rx_vec.max() + rx_vec.min())
        Yb = 0.5 * max_range * np.mgrid[-1:2:2, -1:2:2, -1:2:2][1].flatten() + 0.5 * (ry_vec.max() + ry_vec.min())
        Zb = 0.5 * max_range * np.mgrid[-1:2:2, -1:2:2, -1:2:2][2].flatten() + 0.5 * (rz_vec.max() + rz_vec.min())
        for xb, yb, zb in zip(Xb, Yb, Zb):
            ax.plot([xb], [yb], [zb], 'w')
        ax.scatter(0, 0, 0)
        ax.set_xlabel('$R_x$')
        ax.set_ylabel('$R_y$')
        ax.set_zlabel('$R_z$')
        if arePlotsSaved:
            plt.savefig(paperPath+TheAVSSim.modeRequest+"/boresight_3D.pdf", bbox_inches='tight')
        else:
            plt.title(TheAVSSim.modeRequest + ': bore-sight: 3D')

    def plot_boresightXZ():
        plt.figure(22)
        plt.plot(rx_vec, rz_vec)
        plt.xlabel('$R_X$')
        plt.ylabel('$R_Z$')
        if arePlotsSaved:
            plt.savefig(paperPath+TheAVSSim.modeRequest+"/boresight_XZ.pdf", bbox_inches='tight')
        else:
            plt.title(TheAVSSim.modeRequest + ': bore-sight: XZ-plane')

    alpha = 8.00 * math.pi / 180.0
    rasterSet = np.array([
        [0.0, alpha],
        [0.0, -alpha],
        [0.0, 0.0],
        [alpha, 0.0],
        [-alpha, 0.0],
        [0.0, 0.0],
        [-alpha, alpha],
        [alpha, - alpha],
        [0.0, 0.0],
        [-alpha, -alpha],
        [alpha, alpha],
    ])
    rasterLine_y = rasterSet[:, 0]
    rasterLine_z = rasterSet[:, 1]
    def plot_boresightYZ():
        plt.figure(23)
        plt.plot(psi_vec, theta_vec, 'dodgerblue')
        #plt.plot(rasterLine_y, rasterLine_z, 'darkblue')
        for angleSet in rasterSet:
            if (la.norm(angleSet)!= 0):
                plt.plot(angleSet[0], angleSet[1], 'b')
        plt.xlabel('$R_Y$')
        plt.ylabel('$R_Z$')
        if arePlotsSaved:
            plt.savefig(paperPath+TheAVSSim.modeRequest+"/boresight_YZ.pdf", bbox_inches='tight')
        else:
            plt.title(TheAVSSim.modeRequest + ': bore-sight: YZ-plane')

    #plot_boresightXZ()
    #plot_boresightYZ()
    plot_boresight3D()
    plot321Angles()


def plotTrueBodyEulerSet(sigma_BN):
    rx_vec = np.array([])
    ry_vec = np.array([])
    rz_vec = np.array([])

    psi_vec = np.array([])
    theta_vec = np.array([])
    t = sigma_BN[:, 0] * mc.NANO2SEC / 3600.0
    for i in range(len(t)):
        if t[i] > 0.5:
            e = rbk.MRP2Euler321(sigma_BN[i, 1:])
            psi_vec = np.append(psi_vec, e[0])
            theta_vec = np.append(theta_vec, e[1])
            rx = cos(e[1]) * cos(e[0])
            ry = cos(e[1]) * sin(e[0])
            rz = sin(e[1])
            rx_vec = np.append(rx_vec, rx)
            ry_vec = np.append(ry_vec, ry)
            rz_vec = np.append(rz_vec, rz)

    def plot_real3DBoresight():
        fig = plt.figure(100, figsize=(3.75,3.75))
        ax = fig.add_subplot(111, projection='3d')
        #ax.view_init(elev=0., azim=0.)
        ax.plot(rx_vec, ry_vec, rz_vec, color='magenta')
        max_range = np.array([rx_vec.max() - rx_vec.min(), ry_vec.max() - ry_vec.min(), rz_vec.max() - rz_vec.min()]).max()
        Xb = 0.5 * max_range * np.mgrid[-1:2:2, -1:2:2, -1:2:2][0].flatten() + 0.5 * (rx_vec.max() + rx_vec.min())
        Yb = 0.5 * max_range * np.mgrid[-1:2:2, -1:2:2, -1:2:2][1].flatten() + 0.5 * (ry_vec.max() + ry_vec.min())
        Zb = 0.5 * max_range * np.mgrid[-1:2:2, -1:2:2, -1:2:2][2].flatten() + 0.5 * (rz_vec.max() + rz_vec.min())
        for xb, yb, zb in zip(Xb, Yb, Zb):
            ax.plot([xb], [yb], [zb], 'w')
        ax.scatter(0, 0, 0, color = 'dodgerblue')
        ax.set_xlabel('$R_x$')
        ax.set_ylabel('$R_y$')
        ax.set_zlabel('$R_z$')
        ax.legend(['Bore-sight', 'Spacecraft'])
        if arePlotsSaved:
            plt.savefig(paperPath + TheAVSSim.modeRequest + "/realBoresightPointing.pdf", bbox_inches='tight')
        else:
            plt.title(' Real Body Bore-sight Pointing')

    def plot_real2DBoresight():
        plt.figure(101)
        plt.plot(ry_vec, rz_vec)
        plt.xlabel('$R_Y$')
        plt.ylabel('$R_Z$')
        if arePlotsSaved:
            plt.savefig(paperPath+TheAVSSim.modeRequest+"/realBoresight_YZ.pdf", bbox_inches='tight')
        else:
            plt.title(TheAVSSim.modeRequest + ': Real bore-sight: YZ-plane')

    alpha = 8.00 * math.pi / 180.0
    offAlpha = alpha * 0.5
    rasterLines = np.array([
        [0.0, alpha + offAlpha],
        [0.0, -(alpha + offAlpha)],
        [0.0, 0.0],
        [alpha + offAlpha, 0.0],
        [-(alpha+ offAlpha), 0.0],
        [0.0, 0.0],
        [-(alpha + offAlpha), alpha + offAlpha],
        [alpha + offAlpha, - (alpha + offAlpha)],
        [0.0, 0.0],
        [-(alpha + offAlpha), -(alpha + offAlpha)],
        [alpha + offAlpha, alpha + offAlpha],
    ])

    rasterNominalLines = np.array([
        [0.0, alpha],
        [0.0, -alpha],
        [0.0, 0.0],
        [alpha, 0.0],
        [-alpha, 0.0],
        [0.0, 0.0],
        [-alpha, alpha],
        [alpha, - alpha],
        [0.0, 0.0],
        [-alpha, -alpha],
        [alpha, alpha],
    ])

    rasterNomLine_y = rasterNominalLines[:, 0]
    rasterNomLine_z = rasterNominalLines[:, 1]
    rasterLine_y = rasterLines[:, 0]
    rasterLine_z = rasterLines[:, 1]
    angleSetList = np.array([
        [alpha, 0.0],
        [-alpha, -alpha],
        [alpha, -alpha],
        [0.0, alpha]
    ])
    n_a = ['(1.a)','(2.a)', '(3.a)', '(4.a)']
    n_b = ['(1.b)','(2.b)', '(3.b)', '(4.b)']
    def plot_scanAngles():
        plt.figure(102, figsize=(3.,3.))
        plt.plot(rasterLine_y, rasterLine_z, c='dodgerblue', ls='--')
        plt.plot(psi_vec, theta_vec, 'magenta')
        plt.plot(rasterNomLine_y, rasterNomLine_z, 'b--')

        for i in range(len(rasterLine_y)):
            plt.scatter(rasterNomLine_y[i], rasterNomLine_z[i], s=10, c='b')
        for j, txt_a in enumerate(n_a):
            plt.annotate(txt_a, (angleSetList[j, 0], angleSetList[j, 1]))
        for k, txt_b in enumerate(n_b):
            plt.annotate(txt_b, (-angleSetList[k, 0], -angleSetList[k, 1]))
        plt.xlabel('Yaw Angle, rad')
        plt.ylabel('Pitch Angle, rad')
        plt.legend(['Offset', 'Maneuver', 'Raster'])
        if arePlotsSaved:
            plt.savefig(paperPath+TheAVSSim.modeRequest+"/scanAngles.pdf", bbox_inches='tight')
        else:
            plt.title(TheAVSSim.modeRequest + ': scan angles')

    plot_scanAngles()
    plot_real3DBoresight()
    #plot_real2DBoresight()



def plotTrackingError(sigma_BR, omega_BR_B):
    print 'sigma_BR = ', sigma_BR[:, 1:]
    print 'omega_BR_B = ', omega_BR_B[:, 1:]
    print '\n'
    t = sigma_BR[:, 0] * mc.NANO2SEC / 3600.0
    def plotSigma():
        plt.figure(30)
        plt.plot(t, sigma_BR[:, 1], color_x)
        plt.plot(t, sigma_BR[:, 2], color_y),
        plt.plot(t, sigma_BR[:, 3], color_z)
        plotLabels_sigma()
        if arePlotsSaved:
            plt.savefig(paperPath + TheAVSSim.modeRequest + "/sigma_BR.pdf", bbox_inches='tight')
        else:
            plt.title(TheAVSSim.modeRequest + ': Att Tracking Error $\sigma_{BR}$')

    def plotOmega():
        plt.figure(31)
        plt.plot(t, omega_BR_B[:, 1], color_x)
        plt.plot(t, omega_BR_B[:, 2], color_y)
        plt.plot(t, omega_BR_B[:, 3], color_z)
        plotLabels_omega()
        if arePlotsSaved:
            plt.savefig(paperPath + TheAVSSim.modeRequest + '/omega_BR_B.pdf', bbox_inches='tight')
        else:
            plt.title(TheAVSSim.modeRequest + ': Rate Tracking Error $\omega_{BR, B}$')

    def plotAverageError():
        plt.figure(32)
        t = sigma_BR[:, 0] * mc.NANO2SEC / 3600.0
        eps1 = np.array([])
        eps2 = np.array([])
        eps3 = np.array([])
        for i in range(len(t)):
            e1 = np.sqrt(sigma_BR[i, 1] * sigma_BR[i, 1] + omega_BR_B[i, 1] * omega_BR_B[i, 1])
            e2 = np.sqrt(sigma_BR[i, 2] * sigma_BR[i, 2] + omega_BR_B[i, 2] * omega_BR_B[i, 2])
            e3 = np.sqrt(sigma_BR[i, 3] * sigma_BR[i, 3] + omega_BR_B[i, 3] * omega_BR_B[i, 3])
            eps1 = np.append(eps1, e1)
            eps2 = np.append(eps2, e2)
            eps3 = np.append(eps3, e3)
        plt.semilogy(t, eps1, color_x)
        plt.semilogy(t, eps2, color_y)
        plt.semilogy(t, eps3, color_z)
        plt.legend(['$\epsilon_1$', '$\epsilon_2$', '$\epsilon_3$'])
        plt.xlabel('Time, h')
        plt.ylabel('Logarithmic Average Error log($\epsilon$)')
        if arePlotsSaved:
            plt.savefig(paperPath+TheAVSSim.modeRequest+"/error.pdf", bbox_inches='tight')
        else:
            plt.title(TheAVSSim.modeRequest + ': Average Error $\epsilon_i$')


    def absoluteVec(vec):
        abs_vec1 = np.array([])
        abs_vec2 = np.array([])
        abs_vec3 = np.array([])
        t = vec[:, 0]
        for i in range(len(t)):
            abs_1 = np.abs(vec[i, 1])
            abs_2 = np.abs(vec[i, 2])
            abs_3 = np.abs(vec[i, 3])
            abs_vec1 = np.append(abs_vec1, abs_1)
            abs_vec2 = np.append(abs_vec2, abs_2)
            abs_vec3 = np.append(abs_vec3, abs_3)
        return (abs_vec1, abs_vec2, abs_vec3)


    (absSigma_vec1, absSigma_vec2, absSigma_vec3) = absoluteVec(sigma_BR)
    def plotLogSigma():
        plt.figure(33)
        plt.semilogy(t, absSigma_vec1, color_x)
        plt.semilogy(t, absSigma_vec2, color_y),
        plt.semilogy(t, absSigma_vec3, color_z)
        plt.legend(['log$(\sigma_1)$', 'log$(\sigma_2)$', 'log$(\sigma_3)$'])
        plt.xlabel('Time, h')
        plt.ylabel('Logarithmic MRP Error: log($\epsilon$)')
        if arePlotsSaved:
            plt.savefig(paperPath+TheAVSSim.modeRequest+"/logSigma_BR.pdf", bbox_inches='tight')
        else:
            plt.title(TheAVSSim.modeRequest + ': Log Att Tracking Error log($\sigma_{BR}$)')

    (absOmega_vec1, absOmega_vec2, absOmega_vec3) = absoluteVec(omega_BR_B)
    def plotLogOmega():
        plt.figure(34)
        plt.semilogy(t, absOmega_vec1, color_x)
        plt.semilogy(t, absOmega_vec2, color_y),
        plt.semilogy(t, absOmega_vec3, color_z)
        plt.legend(['log($\omega_1$)', 'log($\omega_2$)', 'log($\omega_3$)'])
        plt.xlabel('Time, h')
        plt.ylabel('Logarithmic Rate Error: log($\omega$)')
        if arePlotsSaved:
            plt.savefig(paperPath+TheAVSSim.modeRequest+"/logOmega_BR.pdf", bbox_inches='tight')
        else:
            plt.title(TheAVSSim.modeRequest + ': Log Rate Error log($\omega_{BR}$)')


    plotSigma()
    plotOmega()
    plotAverageError()
    plotLogSigma()
    plotLogOmega()

def plotControlTorque(Lr):
    print 'Lr = ', Lr[:, 1:]
    print '\n'
    t = Lr[:, 0] * mc.NANO2SEC / 3600.0
    plt.figure(40)
    plt.plot(t, Lr[:, 1], color_x)
    plt.plot(t, Lr[:, 2], color_y)
    plt.plot(t, Lr[:, 3], color_z)
    plt.legend(['$u_1$', '$u_2$', '$u_3$'])
    plt.xlabel('Time, h')
    plt.ylabel('Control Torque, N$\cdot$m')
    if arePlotsSaved:
        plt.savefig(paperPath+TheAVSSim.modeRequest+"/controlTorque.pdf", bbox_inches='tight')
    else:
        plt.title(TheAVSSim.modeRequest + ': Control Torque Request $L_r$')


def plotEffectorTorqueRequest(u):
    print 'u = ', u[:, 1:]
    print '\n'

    t = u[:, 0] * mc.NANO2SEC / 3600.0
    plt.figure(41)
    plt.ylim([-0.2, 0.2])
    plt.plot(t, u[:, 1], color_x)
    plt.plot(t, u[:, 2], color_y)
    plt.plot(t, u[:, 3], color_z)
    plt.plot(t, u[:, 4], 'm')
    plt.legend(['$u_1$', '$u_2$', '$u_3$', '$u_4$'])
    plt.xlabel('Time, h')
    plt.ylabel('Torque, N$\cdot$m')
    if arePlotsSaved:
        plt.savefig(paperPath+TheAVSSim.modeRequest+"/effectorTorque.pdf", bbox_inches='tight')
    else:
        plt.title(TheAVSSim.modeRequest + ': Effector Torque Request $u_r$')

# ------------------- SUPPORT METHODS------------------- #
# MRP Feedback gains
def computeDiscriminant(K, P, I):
    d = np.array([])
    for I_i in I:
        d = np.append(d, -K * I_i + P * P)
    return d
def computeGains(P, I):
    K = np.array([])
    for i in range(3):
        K = np.append(K, P * P / I[i])
    K_cr = min(K)
    return K_cr


# ------------------- MAIN ------------------- #

def executeGuidance(TheAVSSim):
    def doubleTest(mode1, mode2):
        TheAVSSim.modeRequest = mode1
        print '\n Mode Request = ', TheAVSSim.modeRequest
        TheAVSSim.ConfigureStopTime(int(60 * 20 * 1E9))
        TheAVSSim.ExecuteSimulation()

        TheAVSSim.modeRequest = mode2
        print '\n Mode Request = ', TheAVSSim.modeRequest
        TheAVSSim.ConfigureStopTime(int(60 * 40 * 1E9)) # 60 * 20 * 8 * 1E9
        TheAVSSim.ExecuteSimulation()

    def singleTest(mode):
        TheAVSSim.modeRequest = mode
        print '\n Mode Request = ', TheAVSSim.modeRequest
        t_sim = 2 * 60 * 20 * 4
        print 'Sim Time = ', t_sim
        TheAVSSim.ConfigureStopTime(int(t_sim * 1E9))
        #TheAVSSim.ConfigureStopTime(int(60 * 20 * 1E9))
        TheAVSSim.ExecuteSimulation()


    # STAND-ALONE HILL POINT:
    #TheAVSSim.hillPointData.outputDataName = "att_ref_output"

    # STAND-ALONE INERTIAL 3D POINT:
    #TheAVSSim.inertial3DData.outputDataName = "att_ref_output"

    # VELOCITY POINT:
    TheAVSSim.velocityPointData.mu = TheAVSSim.VehOrbElemObject.mu

    # CEL2BDY POINT:
    TheAVSSim.celTwoBodyPointData.inputCelMessName = "mars_display_frame_data"
    #TheAVSSim.celTwoBodyPointData.inputSecMessName = "sun_display_frame_data"

    # EULER ANGLE ROTATION (FOR ORBIT AXIS SPIN)
    angleRates = np.array([0.0, 0.0, 0.3]) * mc.D2R
    #TheAVSSim.eulerRotationData.angleRates = angleRates

    # RASTER MNVR
    TheAVSSim.eulerRotationData.inputEulerSetName = "euler_angle_set"
    TheAVSSim.eulerRotationData.inputEulerRatesName = "euler_angle_rates"

    # ATT TRACKING ERROR
    angleOff = np.pi
    R0R = rbk.Mi(angleOff, 3)
    sigma_R0R = rbk.C2MRP(R0R)
    #TheAVSSim.attTrackingErrorData.sigma_R0R = sigma_R0R

    # DEAD-BAND
    #TheAVSSim.MRP_SteeringRWAData.inputGuidName = "db_att_guid_out"

    # MRP FEEDBACK GAINS
    P = 45
    I_vec = TheAVSSim.VehConfigData.ISCPntB_S
    I = np.array([I_vec[0], I_vec[4], I_vec[8]])
    print 'I = ', I
    K_cr = computeGains(P, I) 
    d = computeDiscriminant(K_cr, P, I)
    T = 2.0 * np.min(I) / P
    print 'Inertia = ', I
    print 'Discriminant = ', d
    print 'K = ', K_cr
    print 'P = ', P
    print 'Decay Time [s] = ', T
    TheAVSSim.MRP_FeedbackRWAData.K = K_cr
    TheAVSSim.MRP_FeedbackRWAData.P = P

    # VISUALIZATION
    #TheAVSSim.isUsingVisualization = True
    #TheAVSSim.clockSynchData.accelFactor = 20.0 * 1.

    # SPICE TIME
    #TheAVSSim.SpiceObject.UTCCalInit = "2015 June 15, 12:00:00.0"

    # INIT SIM:
    TheAVSSim.InitializeSimulation()
    TheAVSSim.ConfigureStopTime(int(1 * 1E9))
    TheAVSSim.ExecuteSimulation()

    # GUIDANCE PROFILES
    #singleTest('inertial3DPoint')
    #doubleTest('inertial3DPoint', 'inertial3DSpin')
    #singleTest('hillPoint')
    #singleTest('velocityPoint')
    #doubleTest('hillPoint', 'velocityPoint')
    #singleTest('celTwoBodyPoint')
    #doubleTest('velocityPoint', 'celTwoBodyPoint')
    #singleTest('inertial3DSpin')
    #singleTest('eulerRotation')
    singleTest('rasterMnvr')
    #singleTest('deadbandGuid')
    #singleTest('rwMotorTorqueControl')

if __name__ == "__main__":
    TheAVSSim = AVSSim.AVSSim()
    TheAVSSim.TotalSim.logThisMessage("controlTorqueRaw", int(1E9))
    TheAVSSim.TotalSim.logThisMessage("att_ref_output_stage1", int(1E9))
    TheAVSSim.TotalSim.logThisMessage("simple_att_nav_output", int(1E9))
    TheAVSSim.TotalSim.logThisMessage("simple_trans_nav_output", int(1E9))
    TheAVSSim.TotalSim.logThisMessage("att_ref_output", int(1E9))
    TheAVSSim.TotalSim.logThisMessage("nom_att_guid_out", int(1E9))
    TheAVSSim.TotalSim.logThisMessage("db_att_guid_out", int(1E9))
    TheAVSSim.TotalSim.logThisMessage("euler_set_output", int(1E9))
    TheAVSSim.TotalSim.logThisMessage("euler_rates_output", int(1E9))
    TheAVSSim.TotalSim.logThisMessage("reactionwheel_cmds", int(1E9))
    TheAVSSim.TotalSim.logThisMessage("rwa_config_data", int(1E9))

    TheAVSSim.VehDynObject.gravData[0].IsCentralBody = False
    TheAVSSim.VehDynObject.gravData[0].IsDisplayBody = False
    TheAVSSim.VehDynObject.gravData[2].IsCentralBody = True
    TheAVSSim.VehDynObject.gravData[2].IsDisplayBody = True
    TheAVSSim.SpiceObject.zeroBase = "mars"
    TheAVSSim.SpiceObject.referenceBase = "MARSIAU"
    TheAVSSim.VehOrbElemObject.mu = TheAVSSim.MarsGravBody.mu

    TheAVSSim.VehOrbElemObject.CurrentElem.a = af.M_radius * 2.2 * 1000.0
    TheAVSSim.VehOrbElemObject.CurrentElem.e = 0.4
    TheAVSSim.VehOrbElemObject.CurrentElem.i = 0.0 * math.pi / 180.0
    TheAVSSim.VehOrbElemObject.CurrentElem.Omega = 0.0
    TheAVSSim.VehOrbElemObject.CurrentElem.omega = 0.0
    TheAVSSim.VehOrbElemObject.CurrentElem.f = -90.0 * math.pi / 180.0

    # Convert those OEs to cartesian
    TheAVSSim.VehOrbElemObject.Elements2Cartesian()
    TheAVSSim.VehDynObject.PositionInit = sim_model.DoubleVector(TheAVSSim.VehOrbElemObject.r_N)
    TheAVSSim.VehDynObject.VelocityInit = sim_model.DoubleVector( TheAVSSim.VehOrbElemObject.v_N)

    executeGuidance(TheAVSSim)

    P = af.orbitalPeriod(TheAVSSim.VehOrbElemObject.CurrentElem.a, TheAVSSim.VehOrbElemObject.mu)
    n = 2 * np.pi / P
    print '\n'
    print 'Orbital SMA [km] = ', TheAVSSim.VehOrbElemObject.CurrentElem.a / 1000.0
    print 'Orbital Period [days] = ', P * af.SEC2DAY
    print 'Orbital Mean Motion [rad/s]', n
    print 'Mars Orbit Eccentricity = ', TheAVSSim.VehOrbElemObject.CurrentElem.e
    print 'Mars Orbit Inclination = ', TheAVSSim.VehOrbElemObject.CurrentElem.i
    print '\n'

    r_BN_N = TheAVSSim.pullMessageLogData("simple_trans_nav_output.r_BN_N", range(3))
    v_BN_N = TheAVSSim.pullMessageLogData("simple_trans_nav_output.v_BN_N", range(3))
    #plotRV_mag(r_BN_N, v_BN_N)

    sigma_BN = TheAVSSim.pullMessageLogData("simple_att_nav_output.sigma_BN", range(3))
    omega_BN_B = TheAVSSim.pullMessageLogData("simple_att_nav_output.omega_BN_B", range(3))
    #plotRotNav(sigma_BN, omega_BN_B)
    if TheAVSSim.modeRequest == 'rasterMnvr':
        plotTrueBodyEulerSet(sigma_BN)


    sigma_RN = TheAVSSim.pullMessageLogData("att_ref_output.sigma_RN", range(3))
    omega_RN_N = TheAVSSim.pullMessageLogData("att_ref_output.omega_RN_N", range(3))
    domega_RN_N = TheAVSSim.pullMessageLogData("att_ref_output.domega_RN_N", range(3))
    plotReference(sigma_RN, omega_RN_N, domega_RN_N)

    if TheAVSSim.modeRequest =='eulerRotation' or TheAVSSim.modeRequest == 'rasterMnvr':
        euler123set = TheAVSSim.pullMessageLogData("euler_set_output.set", range(3))
        euler123rates = TheAVSSim.pullMessageLogData("euler_rates_output.set", range(3))
        plotEulerSet(euler123set)
        plotEulerRates(euler123rates)

    if (TheAVSSim.modeRequest == 'rasterMnvr'
        or TheAVSSim.modeRequest =='eulerRotation'
        or TheAVSSim.modeRequest =='inertial3DSpin'):
        sigma_R0N = TheAVSSim.pullMessageLogData("att_ref_output_stage1.sigma_RN", range(3))
        omega_R0N_N = TheAVSSim.pullMessageLogData("att_ref_output_stage1.omega_RN_N", range(3))
        domega_R0N_N = TheAVSSim.pullMessageLogData("att_ref_output_stage1.domega_RN_N", range(3))
        plotBaseReference(sigma_R0N, omega_R0N_N)


    if (TheAVSSim.modeRequest == 'deadbandGuid'):
        sigma_BR = TheAVSSim.pullMessageLogData("db_att_guid_out.sigma_BR", range(3))
        omega_BR_B = TheAVSSim.pullMessageLogData("db_att_guid_out.omega_BR_B", range(3))
        plotTrackingError(sigma_BR, omega_BR_B)

        dbError = TheAVSSim.GetLogVariableData('errorDeadband.error')
        boolControlOff = TheAVSSim.GetLogVariableData('errorDeadband.boolWasControlOff')
        print 'Control OFF? = ', boolControlOff[:, 1]
        plt.figure(200)
        plt.plot(dbError[:, 0] * 1.0E-9, dbError[:, 1], 'b')
        plt.axhline(TheAVSSim.errorDeadbandData.innerThresh, color='green')
        plt.axhline(TheAVSSim.errorDeadbandData.outerThresh, color='red')
        plt.plot(boolControlOff[:, 0] * 1.0E-9, boolControlOff[:, 1], 'magenta')
        plt.ylim([-0.02, 1.5])
        plt.legend(['error', 'inner thresh', 'outer thresh', 'control (0=ON, 1=OFF)'])
        plt.title('Deadband Mode')

    else:
        sigma_BR = TheAVSSim.pullMessageLogData("nom_att_guid_out.sigma_BR", range(3))
        omega_BR_B = TheAVSSim.pullMessageLogData("nom_att_guid_out.omega_BR_B", range(3))
        omega_RN_B = TheAVSSim.pullMessageLogData("nom_att_guid_out.omega_RN_B", range(3))
        domega_RN_B = TheAVSSim.pullMessageLogData("nom_att_guid_out.domega_RN_B", range(3))
        plotTrackingError(sigma_BR, omega_BR_B)

    Lr = TheAVSSim.pullMessageLogData("controlTorqueRaw.torqueRequestBody", range(3))
    #plotControlTorque(Lr)

    u = TheAVSSim.pullMessageLogData("reactionwheel_cmds.effectorRequest", range(4))
    plotEffectorTorqueRequest(u)


    plt.show()
