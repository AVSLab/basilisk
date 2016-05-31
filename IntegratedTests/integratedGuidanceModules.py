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
import numpy as np
from numpy import linalg as la
import ctypes

import AVSSim
import matplotlib.pyplot as plt
import ctypes
import math
import MessagingAccess
import sim_model
import logging

import astroFunctions as af


def executeGuidance(TheAVSSim):
    TheAVSSim.TotalSim.logThisMessage("att_ref_output", int(1E9))
    TheAVSSim.TotalSim.logThisMessage("simple_nav_output", int(1E9))
    TheAVSSim.TotalSim.logThisMessage("nom_att_guid_out", int(1E9))
    TheAVSSim.AddVariableForLogging('orbitAxisSpin.phi_spin', int(1E9))
    TheAVSSim.AddVariableForLogging('orbitAxisSpin.phi_spin0', int(1E9))
    TheAVSSim.AddVariableForLogging('orbitAxisSpin.currMnvrTime', int(1E9))

    TheAVSSim.InitializeSimulation()
    TheAVSSim.ConfigureStopTime(int(1 * 1E9))
    TheAVSSim.ExecuteSimulation()
    #TheAVSSim.modeRequest = 'inertial3DPoint'
    #TheAVSSim.modeRequest = 'velocityPoint'
    #TheAVSSim.modeRequest = 'celTwoBodyPoint'
    #TheAVSSim.modeRequest = 'hillPoint'
    TheAVSSim.modeRequest = 'orbitAxisSpin'
    TheAVSSim.ConfigureStopTime(int(60 * 10*7 * 1E9))
    TheAVSSim.ExecuteSimulation()

if __name__ == "__main__":
    TheAVSSim = AVSSim.AVSSim()

    TheAVSSim.VehDynObject.GravData[0].IsCentralBody = False
    TheAVSSim.VehDynObject.GravData[0].IsDisplayBody = False
    TheAVSSim.VehDynObject.GravData[2].IsCentralBody = True
    TheAVSSim.VehDynObject.GravData[2].IsDisplayBody = True
    TheAVSSim.SpiceObject.zeroBase = "mars"
    TheAVSSim.SpiceObject.referenceBase = "MARSIAU"
    TheAVSSim.VehOrbElemObject.mu = TheAVSSim.MarsGravBody.mu

    TheAVSSim.isUsingVizualization = True

    TheAVSSim.VehOrbElemObject.CurrentElem.a = 8.0 * af.M_radius * 1000.0
    TheAVSSim.VehOrbElemObject.CurrentElem.e = 0.4
    TheAVSSim.VehOrbElemObject.CurrentElem.i = 0.0
    TheAVSSim.VehOrbElemObject.CurrentElem.Omega = 0.0
    TheAVSSim.VehOrbElemObject.CurrentElem.omega = 0.0
    TheAVSSim.VehOrbElemObject.CurrentElem.f = 0.0 * np.pi / 180.0

    # Convert those OEs to cartesian
    TheAVSSim.VehOrbElemObject.Elements2Cartesian()
    PosVec = ctypes.cast(TheAVSSim.VehOrbElemObject.r_N.__long__(),
                         ctypes.POINTER(ctypes.c_double))
    VelVec = ctypes.cast(TheAVSSim.VehOrbElemObject.v_N.__long__(),
                         ctypes.POINTER(ctypes.c_double))
    def printInitPosVel():
        TheAVSSim.VehDynObject.PositionInit = sim_model.DoubleVector([PosVec[0], PosVec[1], PosVec[2]])
        TheAVSSim.VehDynObject.VelocityInit = sim_model.DoubleVector([VelVec[0], VelVec[1], VelVec[2]])
        print 'PositionInit = ', '(',PosVec[0], ', ', PosVec[1],', ', PosVec[2],')'
        print 'VelocityInit = ', '(',VelVec[0], ', ', VelVec[1],', ', VelVec[2],')'

    executeGuidance(TheAVSSim)
    # print 'PositionFinal = ', '(',PosVec[0], ', ', PosVec[1],', ', PosVec[2],')'
    # print 'VelocityFinal = ', '(',VelVec[0], ', ', VelVec[1],', ', VelVec[2],')'

    def printCelBdyInitPosVel():
        PosMars = ctypes.cast(TheAVSSim.VehDynObject.GravData[2].PosFromEphem.__long__(),
                             ctypes.POINTER(ctypes.c_double))
        VelMars = ctypes.cast(TheAVSSim.VehDynObject.GravData[2].VelFromEphem.__long__(),
                         ctypes.POINTER(ctypes.c_double))
        print 'r_Mars = ', '(', PosMars[0], ', ', PosMars[1],', ', PosMars[2],')'
        print 'v_Mars = ', '(', VelMars[0], ', ', VelMars[1],', ', VelMars[2],')'


    def plotNavState():
        r_BN_N = TheAVSSim.pullMessageLogData("simple_nav_output.r_BN_N", range(3))
        v_BN_N = TheAVSSim.pullMessageLogData("simple_nav_output.v_BN_N", range(3))
        sigma_BN = TheAVSSim.pullMessageLogData("simple_nav_output.sigma_BN", range(3))
        omega_BN = TheAVSSim.pullMessageLogData("simple_nav_output.omega_BN_B", range(3))

        print 'NAV: '
        print 'r_BN_N = ', r_BN_N[:, 1:]
        print '\n'
        print 'v_BN_N = ', v_BN_N[:, 1:]
        print '\n'
        print 'sigma_BN = ', sigma_BN[:, 1:]
        print '\n'

        def plotR_mag():
            plt.figure(0)
            t = r_BN_N[:, 0]
            r = np.array([])
            for i in range(len(t)):
                r = np.append(r, la.norm(r_BN_N[i, 1:]))
            plt.plot(t * 1E-9, r * 1E-3)

        def plotR():
            plt.figure(1)
            plt.plot(r_BN_N[:, 0] * 1E-9, r_BN_N[:, 1], r_BN_N[:, 0] * 1E-9, r_BN_N[:, 2],
                     r_BN_N[:, 0] * 1E-9, r_BN_N[:, 3])
            plt.legend(['$x_1$', '$x_2$', '$x_3$'])
            plt.title('Position: $r_{BN, N}$')

        def plotV():
            plt.figure(2)
            plt.plot(v_BN_N[:, 0] * 1E-9, v_BN_N[:, 1], v_BN_N[:, 0] * 1E-9, v_BN_N[:, 2],
                     v_BN_N[:, 0] * 1E-9, v_BN_N[:, 3])
            plt.legend(['$x_1$', '$x_2$', '$x_3$'])
            plt.title('Velocity: $v_{BN, N}$')

        def plotSigmaBN():
            plt.figure(3)
            plt.ylim([-1.0, 1.0])
            plt.plot(sigma_BN[:, 0] * 1E-9, sigma_BN[:, 1], sigma_BN[:, 0] * 1E-9, sigma_BN[:, 2],
                     sigma_BN[:, 0] * 1E-9, sigma_BN[:, 3])
            plt.legend(['$x_1$', '$x_2$', '$x_3$'])
            plt.title('Att Set: $\sigma_{BN}$')

        def plotOmegaBN():
            plt.figure(4)
            plt.plot(omega_BN[:, 0] * 1E-9, omega_BN[:, 1], omega_BN[:, 0] * 1E-9, omega_BN[:, 2],
                     omega_BN[:, 0] * 1E-9, omega_BN[:, 3])
            plt.legend(['$x_1$', '$x_2$', '$x_3$'])
            plt.title('Att Set: $\omega_{BN}$')
            
        # plotR_mag()
        #plotR()
        # plotV()
        plotSigmaBN()
        plotOmegaBN()

    def plotReference():
        sigma_RN = TheAVSSim.pullMessageLogData("att_ref_output.sigma_RN", range(3))
        omega_RN_N = TheAVSSim.pullMessageLogData("att_ref_output.omega_RN_N", range(3))
        domega_RN_N = TheAVSSim.pullMessageLogData("att_ref_output.domega_RN_N", range(3))
        phi_spin = TheAVSSim.GetLogVariableData('orbitAxisSpin.phi_spin')
        phi_spin0 = TheAVSSim.GetLogVariableData('orbitAxisSpin.phi_spin0')
        currMnvrTime = TheAVSSim.GetLogVariableData('orbitAxisSpin.currMnvrTime')

        print 'sigma_RN = ', sigma_RN[:, 1:]
        print 'omega_RN_N = ', omega_RN_N[:, 1:]
        print 'domega_RN_N = ', domega_RN_N[:, 1:]
        print 'phi_spin = ', phi_spin[1:, 1]
        print 'phi_spin0 = ', phi_spin0[1:, 1]
        print 'currMnvrTime = ', currMnvrTime[1:, 1]

        def plotSigmaRN():
            plt.figure(10)
            plt.ylim([-1.0, 1.0])
            plt.plot(sigma_RN[:, 0] * 1E-9, sigma_RN[:, 1], sigma_RN[:, 0] * 1E-9, sigma_RN[:, 2],
                     sigma_RN[:, 0] * 1E-9, sigma_RN[:, 3])
            plt.legend(['$x_1$', '$x_2$', '$x_3$'])
            plt.title('Att Set: $\sigma_{RN}$')

        def plotOmegaRN():
            plt.figure(11)
            plt.plot(omega_RN_N[:, 0] * 1E-9, omega_RN_N[:, 1], 'b--', omega_RN_N[:, 0] * 1E-9, omega_RN_N[:, 2], 'r--',
                     omega_RN_N[:, 0] * 1E-9, omega_RN_N[:, 3], 'g--',)
            plt.legend(['$x_1$', '$x_2$', '$x_3$'])
            plt.title('Rates: $\omega_{RN, N}$')

        def plotDotOmegaRN():
            plt.figure(12)
            plt.plot(domega_RN_N[:, 0] * 1E-9, domega_RN_N[:, 1], 'b--', domega_RN_N[:, 0] * 1E-9, domega_RN_N[:, 2], 'r--',
                     domega_RN_N[:, 0] * 1E-9, domega_RN_N[:, 3], 'g--', )
            plt.legend(['$x_1$', '$x_2$', '$x_3$'])
            plt.title('Angular acceleration: $\dot\omega_{RN, N}$')

        def plotPhi():
            plt.figure(13)
            plt.plot(phi_spin[1:, 0] * 1E-9, phi_spin[1:, 1], phi_spin0[1:, 0] * 1E-9, phi_spin0[1:, 1])
            plt.legend(['$\phi$', '$\phi_0$'])
            plt.title('Spin angle [rad]')

        def plotMnvrTimes():
            plt.figure(14)
            plt.plot(currMnvrTime[:, 0] * 1E-9, currMnvrTime[:, 1])
            plt.title('Current maneuver time')

        plotSigmaRN()
        plotOmegaRN()
        # plotDotOmegaRN()
        # plotPhi()
        # plotMnvrTimes()

    def plotTrackingError():
        sigma_BR = TheAVSSim.pullMessageLogData("nom_att_guid_out.sigma_BR", range(3))
        omega_BR_B = TheAVSSim.pullMessageLogData("nom_att_guid_out.omega_BR_B", range(3))
        print 'sigma_BR = ', sigma_BR[:, 1:]
        print 'omega_BR_B = ', omega_BR_B[:, 1:]

        def plotSigmaBR():
            plt.figure(20)
            plt.plot(sigma_BR[:, 0] * 1E-9, sigma_BR[:, 1], sigma_BR[:, 0] * 1E-9, sigma_BR[:, 2],
                     sigma_BR[:, 0] * 1E-9, sigma_BR[:, 3])
            plt.ylim([-1.0, 1.0])
            plt.legend(['$x_1$', '$x_2$', '$x_3$'])
            plt.title('Att Set: $\sigma_{BR}$')

        def plotOmegaBR():
            plt.figure(21)
            plt.plot(omega_BR_B[:, 0] * 1E-9, omega_BR_B[:, 1], 'b--', omega_BR_B[:, 0] * 1E-9, omega_BR_B[:, 2], 'r--',
                     omega_BR_B[:, 0] * 1E-9, omega_BR_B[:, 3], 'g--', )
            plt.legend(['$x_1$', '$x_2$', '$x_3$'])
            plt.title('Rates: $\omega_{BR, B}$')

        plotSigmaBR()
        plotOmegaBR()

    def showPlots():
        plotNavState()
        plotReference()
        plotTrackingError()
        plt.show()

    showPlots()