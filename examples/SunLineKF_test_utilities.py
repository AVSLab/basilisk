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
import inspect
import os

import numpy as np
from Basilisk.utilities import unitTestSupport

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
path = path + "/Support"

import matplotlib.pyplot as plt


def StateErrorCovarPlot(x, Pflat, FilterType, show_plots, saveFigures):
    """
    Support method to plot the state error covariances

    :param x: states
    :param Pflat: Variable
    :param FilterType: specifies the filter type
    :param show_plots: flag
    :param saveFigures: flag
    :return:
    """
    nstates = int(np.sqrt(len(Pflat[0,:])-1))

    P = np.zeros([len(Pflat[:,0]),nstates,nstates])
    t= np.zeros(len(Pflat[:,0]))
    for i in range(len(Pflat[:,0])):
        t[i] = x[i, 0]*1E-9
        P[i,:,:] = Pflat[i,1:37].reshape([nstates,nstates])
        for j in range(len(P[0,0,:])):
            P[i,j,j] = np.sqrt(P[i,j,j])

    if nstates == 6:
        plt.figure(num=None, figsize=(10, 10), dpi=80, facecolor='w', edgecolor='k')
        plt.subplot(321)
        plt.plot(t , x[:, 1], "b", label='Error Filter')
        plt.plot(t , 3 * np.sqrt(P[:, 0, 0]), 'r--',  label='Covar Filter')
        plt.plot(t , -3 * np.sqrt(P[:, 0, 0]), 'r--')
        plt.legend(loc='upper right')
        plt.ylabel('$d_x$(m)')
        plt.title('First LOS component')
        plt.grid()

        plt.subplot(322)
        plt.plot(t , x[:, 4], "b")
        plt.plot(t , 3 * np.sqrt(P[:, 3, 3]), 'r--')
        plt.plot(t , -3 * np.sqrt(P[:, 3, 3]), 'r--')
        plt.ylabel(r'$\dot{d}_x$(m)')
        plt.title('First rate component')
        plt.grid()

        plt.subplot(323)
        plt.plot(t , x[:, 2], "b")
        plt.plot(t , 3 * np.sqrt(P[:, 1, 1]), 'r--')
        plt.plot(t , -3 * np.sqrt(P[:, 1, 1]), 'r--')
        plt.ylabel(r'$d_y$(m)')
        plt.title('Second LOS component')
        plt.grid()

        plt.subplot(324)
        plt.plot(t , x[:, 5], "b")
        plt.plot(t , 3 * np.sqrt(P[:, 4, 4]), 'r--')
        plt.plot(t , -3 * np.sqrt(P[:, 4, 4]), 'r--')
        plt.ylabel(r'$\dot{d}_y$(m)')
        plt.title('Second rate component')
        plt.grid()

        plt.subplot(325)
        plt.plot(t , x[:, 3], "b")
        plt.plot(t , 3 * np.sqrt(P[:, 2, 2]), 'r--')
        plt.plot(t , -3 * np.sqrt(P[:, 2, 2]), 'r--')
        plt.ylabel('$d_z$(m)')
        plt.xlabel('t(s)')
        plt.title('Third LOS component')
        plt.grid()

        if FilterType == 'SuKF':
            plt.subplot(326)
            plt.plot(t, x[:, 6], "b")
            plt.plot(t,  x[:, 6] + 3 * np.sqrt(P[:, 5, 5]), 'r--')
            plt.plot(t,  x[:, 6] -3 * np.sqrt(P[:, 5, 5]), 'r--')
            plt.ylabel(r'$\dot{d}_z$(m)')
            plt.xlabel('t(s)')
            plt.title('Sun Intensity')
            plt.grid()
        else:
            plt.subplot(326)
            plt.plot(t , x[:, 6], "b")
            plt.plot(t , 3 * np.sqrt(P[:, 5, 5]), 'r--')
            plt.plot(t , -3 * np.sqrt(P[:, 5, 5]), 'r--')
            plt.ylabel(r'$\dot{d}_z$(m)')
            plt.xlabel('t(s)')
            plt.title('Third rate component')
            plt.grid()

    if nstates == 3:
        plt.figure(num=None, figsize=(10, 10), dpi=80, facecolor='w', edgecolor='k')
        plt.subplot(311)
        plt.plot(t, x[:, 1], "b", label='Error Filter')
        plt.plot(t, 3 * np.sqrt(P[:, 0, 0]), 'r--', label='Covar Filter')
        plt.plot(t, -3 * np.sqrt(P[:, 0, 0]), 'r--')
        plt.legend(loc='lower right')
        plt.ylabel('$d_x$(m)')
        plt.title('First LOS component')
        plt.grid()

        plt.subplot(312)
        plt.plot(t, x[:, 2], "b")
        plt.plot(t, 3 * np.sqrt(P[:, 1, 1]), 'r--')
        plt.plot(t, -3 * np.sqrt(P[:, 1, 1]), 'r--')
        plt.ylabel('$d_y$(m)')
        plt.title('Second LOST component')
        plt.grid()

        plt.subplot(313)
        plt.plot(t, x[:, 3], "b")
        plt.plot(t, 3 * np.sqrt(P[:, 2, 2]), 'r--')
        plt.plot(t, -3 * np.sqrt(P[:, 2, 2]), 'r--')
        plt.ylabel('$d_z$(m)')
        plt.title('Third LOS component')
        plt.grid()

    if nstates == 5:
        plt.figure(num=None, figsize=(10, 10), dpi=80, facecolor='w', edgecolor='k')
        plt.subplot(321)
        plt.plot(t , x[:, 1], "b", label='Error Filter')
        plt.plot(t , 3 * np.sqrt(P[:, 0, 0]), 'r--',  label='Covar Filter')
        plt.plot(t , -3 * np.sqrt(P[:, 0, 0]), 'r--')
        plt.legend(loc='lower right')
        plt.ylabel('$d_x$(m)')
        plt.title('First LOS component')
        plt.grid()

        plt.subplot(323)
        plt.plot(t , x[:, 2], "b")
        plt.plot(t , 3 * np.sqrt(P[:, 1, 1]), 'r--')
        plt.plot(t , -3 * np.sqrt(P[:, 1, 1]), 'r--')
        plt.ylabel('$d_y$(m)')
        plt.title('Second LOS component')
        plt.grid()

        plt.subplot(324)
        plt.plot(t , x[:, 3], "b")
        plt.plot(t , 3 * np.sqrt(P[:, 3, 3]), 'r--')
        plt.plot(t , -3 * np.sqrt(P[:, 3, 3]), 'r--')
        plt.ylabel(r'$\omega_y$(m)')
        plt.title('Second rate component')
        plt.grid()

        plt.subplot(325)
        plt.plot(t , x[:, 3], "b")
        plt.plot(t , 3 * np.sqrt(P[:, 2, 2]), 'r--')
        plt.plot(t , -3 * np.sqrt(P[:, 2, 2]), 'r--')
        plt.ylabel('$d_z$(m)')
        plt.xlabel('t(s)')
        plt.title('Third LOS component')
        plt.grid()

        plt.subplot(326)
        plt.plot(t , x[:, 5], "b")
        plt.plot(t , 3 * np.sqrt(P[:, 4, 4]), 'r--')
        plt.plot(t , -3 * np.sqrt(P[:, 4, 4]), 'r--')
        plt.ylabel(r'$\omega_z$(m)')
        plt.xlabel('t(s)')
        plt.title('Third rate component')
        plt.grid()

    if saveFigures:
        unitTestSupport.saveScenarioFigure('scenario_Filters_StatesPlot'+FilterType,  plt,  path)
    if show_plots:
        plt.show()
    plt.close('all')


def StatesPlotCompare(x, x2, Pflat, Pflat2, FilterType, show_plots, saveFigures):

    nstates = int(np.sqrt(len(Pflat[0,:])-1))

    P = np.zeros([len(Pflat[:,0]),nstates,nstates])
    P2 = np.zeros([len(Pflat[:,0]),nstates,nstates])
    t= np.zeros(len(Pflat[:,0]))
    for i in range(len(Pflat[:,0])):
        t[i] = x[i, 0]*1E-9
        P[i,:,:] = Pflat[i,1:(nstates*nstates +1)].reshape([nstates,nstates])
        P2[i, :, :] = Pflat2[i, 1:(nstates*nstates +1)].reshape([nstates, nstates])

    if nstates == 6:

        plt.figure(num=None, figsize=(10, 10), dpi=80, facecolor='w', edgecolor='k')
        plt.subplot(321)
        plt.plot(t[0:30] , x[0:30, 1], "b", label='Error Filter')
        plt.plot(t[0:30] , 3 * np.sqrt(P[0:30, 0, 0]), 'r--',  label='Covar Filter')
        plt.plot(t[0:30] , -3 * np.sqrt(P[0:30, 0, 0]), 'r--')
        plt.plot(t[0:30] , x2[0:30, 1], "g", label='Error Expected')
        plt.plot(t[0:30] , 3 * np.sqrt(P2[0:30, 0, 0]), 'c--', label='Covar Expected')
        plt.plot(t[0:30] , -3 * np.sqrt(P2[0:30, 0, 0]), 'c--')
        plt.legend(loc='lower right')
        plt.ylabel('$d_x$(m)')
        plt.title('First LOS component')
        plt.grid()

        plt.subplot(322)
        plt.plot(t[0:30] , x[0:30, 4], "b")
        plt.plot(t[0:30] , 3 * np.sqrt(P[0:30, 3, 3]), 'r--')
        plt.plot(t[0:30] , -3 * np.sqrt(P[0:30, 3, 3]), 'r--')
        plt.plot(t[0:30] , x2[0:30, 4], "g")
        plt.plot(t[0:30] , 3 * np.sqrt(P2[0:30, 3, 3]), 'c--')
        plt.plot(t[0:30] , -3 * np.sqrt(P2[0:30, 3, 3]), 'c--')
        plt.ylabel(r'$\dot{d}_x$(m)')
        plt.title('First rate component')
        plt.grid()

        plt.subplot(323)
        plt.plot(t[0:30] , x[0:30, 2], "b")
        plt.plot(t[0:30] , 3 * np.sqrt(P[0:30, 1, 1]), 'r--')
        plt.plot(t[0:30] , -3 * np.sqrt(P[0:30, 1, 1]), 'r--')
        plt.plot(t[0:30] , x2[0:30, 2], "g")
        plt.plot(t[0:30] , 3 * np.sqrt(P2[0:30, 1, 1]), 'c--')
        plt.plot(t[0:30] , -3 * np.sqrt(P2[0:30, 1, 1]), 'c--')
        plt.ylabel('$d_y$(m)')
        plt.title('Second LOS component')
        plt.grid()

        plt.subplot(324)
        plt.plot(t[0:30] , x[0:30, 5], "b")
        plt.plot(t[0:30] , 3 * np.sqrt(P[0:30, 4, 4]), 'r--')
        plt.plot(t[0:30] , -3 * np.sqrt(P[0:30, 4, 4]), 'r--')
        plt.plot(t[0:30] , x2[0:30, 5], "g")
        plt.plot(t[0:30] , 3 * np.sqrt(P2[0:30, 4, 4]), 'c--')
        plt.plot(t[0:30] , -3 * np.sqrt(P2[0:30, 4, 4]), 'c--')
        plt.ylabel(r'$\dot{d}_y$(m)')
        plt.title('Second rate component')
        plt.grid()

        plt.subplot(325)
        plt.plot(t[0:30] , x[0:30, 3], "b")
        plt.plot(t[0:30] , 3 * np.sqrt(P[0:30, 2, 2]), 'r--')
        plt.plot(t[0:30] , -3 * np.sqrt(P[0:30, 2, 2]), 'r--')
        plt.plot(t[0:30] , x2[0:30, 3], "g")
        plt.plot(t[0:30] , 3 * np.sqrt(P2[0:30, 2, 2]), 'c--')
        plt.plot(t[0:30] , -3 * np.sqrt(P2[0:30, 2, 2]), 'c--')
        plt.ylabel('$d_z$(m)')
        plt.xlabel('t(s)')
        plt.title('Third LOS component')
        plt.grid()


        if FilterType == 'SuKF':
            plt.subplot(326)
            plt.plot(t[0:30], x[0:30, 6], "b")
            plt.plot(t[0:30], x[0:30, 6] + 3 * np.sqrt(P[0:30, 5, 5]), 'r--')
            plt.plot(t[0:30], x[0:30, 6] -3 * np.sqrt(P[0:30, 5, 5]), 'r--')
            plt.plot(t[0:30], x2[0:30, 6], "g")
            plt.plot(t[0:30], x2[0:30, 6]+ 3 * np.sqrt(P2[0:30, 5, 5]), 'c--')
            plt.plot(t[0:30], x2[0:30, 6]-3 * np.sqrt(P2[0:30, 5, 5]), 'c--')
            plt.ylabel('S')
            plt.xlabel('t(s)')
            plt.title('Solar Intensity')
            plt.grid()
        else:
            plt.subplot(326)
            plt.plot(t[0:30] , x[0:30, 6], "b")
            plt.plot(t[0:30] , 3 * np.sqrt(P[0:30, 5, 5]), 'r--')
            plt.plot(t[0:30] , -3 * np.sqrt(P[0:30, 5, 5]), 'r--')
            plt.plot(t[0:30] , x2[0:30, 6], "g")
            plt.plot(t[0:30] , 3 * np.sqrt(P2[0:30, 5, 5]), 'c--')
            plt.plot(t[0:30] , -3 * np.sqrt(P2[0:30, 5, 5]), 'c--')
            plt.ylabel(r'$\dot{d}_z$(m)')
            plt.xlabel('t(s)')
            plt.title('Third rate component')
            plt.grid()

    if nstates == 3:
        plt.figure(num=None, figsize=(10, 10), dpi=80, facecolor='w', edgecolor='k')
        plt.subplot(311)
        plt.plot(t[0:30], x[0:30, 1], "b", label='Error Filter')
        plt.plot(t[0:30], 3 * np.sqrt(P[0:30, 0, 0]), 'r--', label='Covar Filter')
        plt.plot(t[0:30], -3 * np.sqrt(P[0:30, 0, 0]), 'r--')
        plt.plot(t[0:30], x2[0:30, 1], "g", label='Error Expected')
        plt.plot(t[0:30], 3 * np.sqrt(P2[0:30, 0, 0]), 'c--', label='Covar Expected')
        plt.plot(t[0:30], -3 * np.sqrt(P2[0:30, 0, 0]), 'c--')
        plt.ylabel('$d_x$(m)')
        plt.legend(loc='lower right')
        plt.title('First LOS component')
        plt.grid()

        plt.subplot(312)
        plt.plot(t[0:30], x[0:30, 2], "b")
        plt.plot(t[0:30], 3 * np.sqrt(P[0:30, 1, 1]), 'r--')
        plt.plot(t[0:30], -3 * np.sqrt(P[0:30, 1, 1]), 'r--')
        plt.plot(t[0:30], x2[0:30, 2], "g")
        plt.plot(t[0:30], 3 * np.sqrt(P2[0:30, 1, 1]), 'c--')
        plt.plot(t[0:30], -3 * np.sqrt(P2[0:30, 1, 1]), 'c--')
        plt.ylabel('$d_y$(m)')
        plt.title('Second LOS component')
        plt.grid()

        plt.subplot(313)
        plt.plot(t[0:30], x[0:30, 3], "b")
        plt.plot(t[0:30], 3 * np.sqrt(P[0:30, 2, 2]), 'r--')
        plt.plot(t[0:30], -3 * np.sqrt(P[0:30, 2, 2]), 'r--')
        plt.plot(t[0:30], x2[0:30, 3], "g")
        plt.plot(t[0:30], 3 * np.sqrt(P2[0:30, 2, 2]), 'c--')
        plt.plot(t[0:30], -3 * np.sqrt(P2[0:30, 2, 2]), 'c--')
        plt.ylabel('$d_z$(m)')
        plt.title('Third LOS component')
        plt.grid()

    if nstates == 5:

        plt.figure(num=None, figsize=(10, 10), dpi=80, facecolor='w', edgecolor='k')
        plt.subplot(321)
        plt.plot(t[0:30] , x[0:30, 1], "b", label='Error Filter')
        plt.plot(t[0:30] , 3 * np.sqrt(P[0:30, 0, 0]), 'r--',  label='Covar Filter')
        plt.plot(t[0:30] , -3 * np.sqrt(P[0:30, 0, 0]), 'r--')
        plt.plot(t[0:30] , x2[0:30, 1], "g", label='Error Expected')
        plt.plot(t[0:30] , 3 * np.sqrt(P2[0:30, 0, 0]), 'c--', label='Covar Expected')
        plt.plot(t[0:30] , -3 * np.sqrt(P2[0:30, 0, 0]), 'c--')
        plt.legend(loc='lower right')
        plt.ylabel('$d_x$(m)')
        plt.title('First LOS component')
        plt.grid()

        plt.subplot(323)
        plt.plot(t[0:30] , x[0:30, 2], "b")
        plt.plot(t[0:30] , 3 * np.sqrt(P[0:30, 1, 1]), 'r--')
        plt.plot(t[0:30] , -3 * np.sqrt(P[0:30, 1, 1]), 'r--')
        plt.plot(t[0:30] , x2[0:30, 2], "g")
        plt.plot(t[0:30] , 3 * np.sqrt(P2[0:30, 1, 1]), 'c--')
        plt.plot(t[0:30] , -3 * np.sqrt(P2[0:30, 1, 1]), 'c--')
        plt.ylabel('$d_y$(m)')
        plt.title('Second LOS component')
        plt.grid()

        plt.subplot(324)
        plt.plot(t[0:30] , x[0:30, 4], "b")
        plt.plot(t[0:30] , 3 * np.sqrt(P[0:30, 3, 3]), 'r--')
        plt.plot(t[0:30] , -3 * np.sqrt(P[0:30, 3, 3]), 'r--')
        plt.plot(t[0:30] , x2[0:30, 4], "g")
        plt.plot(t[0:30] , 3 * np.sqrt(P2[0:30, 3, 3]), 'c--')
        plt.plot(t[0:30] , -3 * np.sqrt(P2[0:30, 3, 3]), 'c--')
        plt.ylabel(r'$\omega_y$(m)')
        plt.title('Second rate component')
        plt.grid()

        plt.subplot(325)
        plt.plot(t[0:30] , x[0:30, 3], "b")
        plt.plot(t[0:30] , 3 * np.sqrt(P[0:30, 2, 2]), 'r--')
        plt.plot(t[0:30] , -3 * np.sqrt(P[0:30, 2, 2]), 'r--')
        plt.plot(t[0:30] , x2[0:30, 3], "g")
        plt.plot(t[0:30] , 3 * np.sqrt(P2[0:30, 2, 2]), 'c--')
        plt.plot(t[0:30] , -3 * np.sqrt(P2[0:30, 2, 2]), 'c--')
        plt.ylabel('$d_z$(m)')
        plt.xlabel('t(s)')
        plt.title('Third LOS component')
        plt.grid()

        plt.subplot(326)
        plt.plot(t[0:30] , x[0:30, 5], "b")
        plt.plot(t[0:30] , 3 * np.sqrt(P[0:30, 4, 4]), 'r--')
        plt.plot(t[0:30] , -3 * np.sqrt(P[0:30, 4, 4]), 'r--')
        plt.plot(t[0:30] , x2[0:30, 5], "g")
        plt.plot(t[0:30] , 3 * np.sqrt(P2[0:30, 4, 4]), 'c--')
        plt.plot(t[0:30] , -3 * np.sqrt(P2[0:30, 4, 4]), 'c--')
        plt.ylabel(r'$\omega_z$(m)')
        plt.xlabel('t(s)')
        plt.title('Third rate component')
        plt.grid()

    if saveFigures:
        unitTestSupport.saveScenarioFigure('scenario_Filters_StatesCompare'+FilterType, plt, path)

    if show_plots:
        plt.show()
    plt.close()

def numMeasurements(numObs, FilterType, show_plots, saveFigures):
    plt.plot(111)
    plt.plot(numObs[:,0]*(1E-9) , numObs[:, 1], "b")
    plt.ylim([0,5])
    plt.xlabel('t(s)')
    plt.title('Number of Activated CSS')

    if saveFigures:
        unitTestSupport.saveScenarioFigure('scenario_Filters_Obs'+ FilterType, plt,  path)

    if show_plots:
        plt.show()
    plt.close()

def PostFitResiduals(Res, noise, FilterType, show_plots, saveFigures):

    MeasNoise = np.zeros(len(Res[:,0]))
    t= np.zeros(len(Res[:,0]))
    constantVal = np.array([np.nan]*4)
    for i in range(len(Res[:,0])):
        t[i] = Res[i, 0]*1E-9
        MeasNoise[i] = 3*noise
        # Don't plot constant values, they mean no measurement is taken
        if i>0:
            for j in range(1,5):
                with np.errstate(invalid='ignore'):
                    constantRes = np.abs(Res[i,j]-Res[i-1,j])
                    if constantRes < 1E-10 or np.abs(constantVal[j-1] - Res[i,j])<1E-10:
                        constantVal[j-1] = Res[i, j]
                        Res[i, j] = np.nan

    plt.figure(num=None, figsize=(10, 10), dpi=80, facecolor='w', edgecolor='k')
    plt.subplot(411)
    plt.plot(t , Res[:, 1], "b.", label='Residual')
    plt.plot(t , MeasNoise, 'r--', label='Covar')
    plt.plot(t , -MeasNoise, 'r--')
    plt.legend(loc='lower right')
    plt.ylabel('$r_1$(m)')
    plt.ylim([-5*noise, 5*noise])
    plt.title('First CSS')


    plt.subplot(412)
    plt.plot(t , Res[:, 2], "b.")
    plt.plot(t , MeasNoise, 'r--')
    plt.plot(t , -MeasNoise, 'r--')
    plt.ylabel('$r_2$(m)')
    plt.ylim([-5*noise, 5*noise])
    plt.title('Second CSS')

    plt.subplot(413)
    plt.plot(t , Res[:, 3], "b.")
    plt.plot(t , MeasNoise, 'r--')
    plt.plot(t , -MeasNoise, 'r--')
    plt.ylabel('$r_3$(m)')
    plt.ylim([-5*noise, 5*noise])
    plt.title('Third CSS')

    plt.subplot(414)
    plt.plot(t , Res[:, 4], "b.")
    plt.plot(t , MeasNoise, 'r--')
    plt.plot(t , -MeasNoise, 'r--')
    plt.ylim([-5*noise, 5*noise])
    plt.ylabel('$r_4$(m)')
    plt.xlabel('t(s)')
    plt.title('Fourth CSS')

    if saveFigures:
        unitTestSupport.saveScenarioFigure('scenario_Filters_PostFit'+ FilterType, plt,  path)

    if show_plots:
        plt.show()
    plt.close()

def StatesVsExpected(stateLog, Pflat, expectedStateArray, FilterType, show_plots, saveFigures):

    nstates = int(np.sqrt(len(Pflat[0,:])-1))

    P = np.zeros([len(Pflat[:, 0]), nstates, nstates])
    for i in range(len(Pflat[:, 0])):
        P[i, :, :] = Pflat[i, 1:(nstates*nstates +1)].reshape([nstates, nstates])
        for j in range(len(P[0,0,:])):
            P[i,j,j] = np.sqrt(P[i,j,j])

    if nstates ==6:
        plt.figure(num=None, figsize=(10, 10), dpi=80, facecolor='w', edgecolor='k')
        plt.subplot(321)
        plt.plot(stateLog[:, 0] * 1.0E-9, expectedStateArray[:,  1], 'k--', label='Expected')
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:,  1], 'b', label='Filter')
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:,  1] + P[:,0,0], 'r--')
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:,  1] - P[:,0,0], 'r--', label='Covar')
        plt.legend(loc='lower right')
        plt.ylabel('$d_x$(m)')
        plt.title('First LOS component')
        plt.grid()

        plt.subplot(322)
        plt.plot(stateLog[:, 0] * 1.0E-9, expectedStateArray[:,  4], 'k--')
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:,  4], 'b')
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:,  4] + P[:,3,3], 'r--')
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:,  4] - P[:,3,3], 'r--', label='Covar')
        plt.ylabel(r'$\dot{d}_x$(m)')
        plt.title('First rate component')
        plt.grid()

        plt.subplot(323)
        plt.plot(stateLog[:, 0] * 1.0E-9, expectedStateArray[:,  2], 'k--')
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:,  2], 'b')
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:,  2] + P[:,1,1], 'r--')
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:,  2] - P[:,1,1], 'r--', label='Covar')
        plt.ylabel('$d_y$(m)')
        plt.title('Second LOS component')
        plt.grid()

        plt.subplot(324)
        plt.plot(stateLog[:, 0] * 1.0E-9, expectedStateArray[:,  5], 'k--')
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:,  5], 'b')
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:,  5] + P[:,4,4], 'r--')
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:,  5] - P[:,4,4], 'r--', label='Covar')
        plt.ylabel(r'$\dot{d}_y$(m)')
        plt.title('Second rate component')
        plt.grid()

        plt.subplot(325)
        plt.plot(stateLog[:, 0] * 1.0E-9, expectedStateArray[:,  3], 'k--')
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:,  3], 'b')
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:,  3] + P[:,2,2], 'r--')
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:,  3] - P[:,2,2], 'r--', label='Covar')
        plt.ylabel('$d_z$(m)')
        plt.xlabel('t(s)')
        plt.title('Third LOS component')
        plt.grid()

        plt.subplot(326)
        plt.plot(stateLog[:, 0] * 1.0E-9, expectedStateArray[:,  6], 'k--')
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:,  6], 'b')
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:,  6] + P[:,5,5], 'r--')
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:,  6] - P[:,5,5], 'r--', label='Covar')
        plt.ylabel(r'$\dot{d}_z$(m)')
        plt.xlabel('t(s)')
        plt.title('Third rate component')
        plt.grid()

    if nstates ==3:
        plt.figure(num=None, figsize=(10, 10), dpi=80, facecolor='w', edgecolor='k')
        plt.subplot(311)
        plt.plot(stateLog[:, 0] * 1.0E-9, expectedStateArray[:, 1], 'k--', label='Expected')
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:, 1], 'b', label='Filter')
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:, 1] + P[:, 0, 0], 'r--')
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:, 1] - P[:, 0, 0], 'r--', label='Covar')
        plt.ylabel('$d_x$(m)')
        plt.legend(loc='lower right')
        plt.title('First LOS component')
        plt.grid()

        plt.subplot(312)
        plt.plot(stateLog[:, 0] * 1.0E-9, expectedStateArray[:, 2], 'k--')
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:, 2], 'b')
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:, 2] + P[:, 1, 1], 'r--')
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:, 2] - P[:, 1, 1], 'r--', label='Covar')
        plt.ylabel('$d_y$(m)')
        plt.title('Second LOS component')
        plt.grid()

        plt.subplot(313)
        plt.plot(stateLog[:, 0] * 1.0E-9, expectedStateArray[:, 3], 'k--')
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:, 3], 'b')
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:, 3] + P[:, 2, 2], 'r--')
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:, 3] - P[:, 2, 2], 'r--', label='Covar')
        plt.ylabel('$d_z$(m)')
        plt.xlabel('t(s)')
        plt.title('Third LOS component')
        plt.grid()

    if nstates ==5:
        plt.figure(num=None, figsize=(10, 10), dpi=80, facecolor='w', edgecolor='k')
        plt.subplot(321)
        plt.plot(stateLog[:, 0] * 1.0E-9, expectedStateArray[:,  1], 'k--', label='Expected')
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:,  1], 'b', label='Filter')
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:,  1] + P[:,0,0], 'r--')
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:,  1] - P[:,0,0], 'r--', label='Covar')
        plt.legend(loc='lower right')
        plt.ylabel('$d_x$(m)')
        plt.title('First LOS component')
        plt.grid()

        plt.subplot(323)
        plt.plot(stateLog[:, 0] * 1.0E-9, expectedStateArray[:,  2], 'k--')
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:,  2], 'b')
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:,  2] + P[:,1,1], 'r--')
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:,  2] - P[:,1,1], 'r--', label='Covar')
        plt.ylabel('$d_y$(m)')
        plt.title('Second LOS component')
        plt.grid()

        plt.subplot(324)
        plt.plot(stateLog[:, 0] * 1.0E-9, expectedStateArray[:,  4], 'k--')
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:,  4], 'b')
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:,  4] + P[:,3,3], 'r--')
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:,  4] - P[:,3,3], 'r--', label='Covar')
        plt.ylabel(r'$\omega_y$(m)')
        plt.title('Second rate component')
        plt.grid()

        plt.subplot(325)
        plt.plot(stateLog[:, 0] * 1.0E-9, expectedStateArray[:,  3], 'k--')
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:,  3], 'b')
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:,  3] + P[:,2,2], 'r--')
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:,  3] - P[:,2,2], 'r--', label='Covar')
        plt.ylabel('$d_z$(m)')
        plt.xlabel('t(s)')
        plt.title('Third LOS component')
        plt.grid()

        plt.subplot(326)
        plt.plot(stateLog[:, 0] * 1.0E-9, expectedStateArray[:,  5], 'k--')
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:,  5], 'b')
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:,  5] + P[:,4,4], 'r--')
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:,  5] - P[:,4,4], 'r--', label='Covar')
        plt.ylabel(r'$\omega_z$(m)')
        plt.xlabel('t(s)')
        plt.title('Third rate component')
        plt.grid()


    if saveFigures:
        unitTestSupport.saveScenarioFigure('scenario_Filters_StatesExpected' + FilterType , plt, path)

    if show_plots:
        plt.show()
    plt.close()


def StatesVsTargets(target1, target2, stateLog, FilterType, show_plots, saveFigures):

    nstates = int(stateLog[0,:])

    target = np.ones([len(stateLog[:, 0]),nstates])
    for i in range((len(stateLog[:, 0])-1)/2):
        target[i, :] = target1
        target[i+(len(stateLog[:, 0]) - 1) / 2,:] = target2

    if nstates == 6:
        plt.figure(num=None, figsize=(10, 10), dpi=80, facecolor='w', edgecolor='k')
        plt.subplot(321)
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:, 1], 'b', label='Filter')
        plt.plot(stateLog[:, 0] * 1.0E-9, target[:, 0], 'r--', label='Expected')
        plt.legend(loc='lower right')
        plt.title('First LOS component')
        plt.grid()

        plt.subplot(322)
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:, 4], 'b')
        plt.plot(stateLog[:, 0] * 1.0E-9, target[:, 3], 'r--')
        plt.title('First rate component')
        plt.grid()

        plt.subplot(323)
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:, 2], 'b')
        plt.plot(stateLog[:, 0] * 1.0E-9, target[:, 1], 'r--')
        plt.title('Second LOS component')
        plt.grid()

        plt.subplot(324)
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:, 5], 'b')
        plt.plot(stateLog[:, 0] * 1.0E-9, target[:, 4], 'r--')
        plt.title('Second rate component')
        plt.grid()

        plt.subplot(325)
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:, 3], 'b')
        plt.plot(stateLog[:, 0] * 1.0E-9, target[:, 2], 'r--')
        plt.xlabel('t(s)')
        plt.title('Third LOS component')
        plt.grid()

        plt.subplot(326)
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:, 6], 'b')
        plt.plot(stateLog[:, 0] * 1.0E-9, target[:, 5], 'r--')
        plt.xlabel('t(s)')
        plt.title('Third rate component')
        plt.grid()

    if nstates == 3:
        plt.figure(num=None, figsize=(10, 10), dpi=80, facecolor='w', edgecolor='k')
        plt.subplot(311)
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:, 1], 'b', label='Filter')
        plt.plot(stateLog[:, 0] * 1.0E-9, target[:, 0], 'r--', label='Expected')
        plt.legend(loc='lower right')
        plt.title('First LOS component')
        plt.grid()

        plt.subplot(312)
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:, 2], 'b')
        plt.plot(stateLog[:, 0] * 1.0E-9, target[:, 1], 'r--')
        plt.title('Second rate component')
        plt.grid()

        plt.subplot(313)
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:, 3], 'b')
        plt.plot(stateLog[:, 0] * 1.0E-9, target[:, 2], 'r--')
        plt.title('Third LOS component')
        plt.grid()

    if saveFigures:
        unitTestSupport.saveScenarioFigure('scenario_Filters_StatesTarget' + FilterType,  plt,  path)

    if show_plots:
        plt.show()
    plt.close()
