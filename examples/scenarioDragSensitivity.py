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

r"""
Overview
--------

This script executes a parametric analysis of the control law examined in :ref:`scenarioDragRendezvous`,
considering the performance of that control
law across both increasing initial displacements and variations in atmospheric density. 

This script is found in the folder ``src/examples`` and executed by using::

      python3 scenarioDragSensitivity.py

The simulation layout is identical to that used in :ref:`scenarioDragRendezvous`. The simulator used in
that scenario is run using a grid
of true anomaly offsets and atmospheric densities using Python's multiprocessing library, and a
set of surface plots reflecting the controls' terminal performance
and trajectories are produced.

Illustration of Simulation Results
----------------------------------

In this scenario, the differential drag scenario used in :ref:`scenarioDragRendezvous` is examined
across a range of initial along-track orbit offsets and atmospheric densities.
The resulting Hill-frame trajectories corresponding to every fifth simulation run are shown in the following image.

.. image:: /_images/Scenarios/scenarioDragSensitivity_hillTrajectories.svg
   :align: center

To visualize the sensitivity of terminal position and velocity errors to both increasing baseline and
variations in density, the following surface plots - which show the
scale of terminal errors as a function of atmospheric density and maneuver baseline - are shown below:

.. image:: /_images/Scenarios/scenarioDragSensitivity_positionSensitivity.svg
   :align: center

.. image:: /_images/Scenarios/scenarioDragSensitivity_velocitySensitivity.svg
   :align: center

"""

import multiprocessing as mp
import os
import pickle

import numpy as np
from matplotlib import pyplot as plt

from scenarioDragRendezvous import drag_simulator

fileName = os.path.basename(os.path.splitext(__file__)[0])

def sim_wrapper(arg):
    """
     Because multiprocessing.map only considers single-input functions as targets for maps,
     this function maps args and kwargs from the dicts provided by multiprocessing to the inputs
     demanded by drag_simulator.
     """
    args, kwargs = arg

    result = drag_simulator(*args, **kwargs)

    return result


def drag_sensitivity_analysis(ctrlType, nuOffsetNum, densityNum, rerunSims=False):

    alt_offsets= [0]  # np.arange(-100,100,10)
    nu_offsets = np.arange(0.001, 1, (1-0.001)/nuOffsetNum)
    density_multipliers = np.logspace(-1, 0.4, num=densityNum)
    pool = mp.Pool(mp.cpu_count())

    X,Y,Z = np.meshgrid(alt_offsets, nu_offsets, density_multipliers)

    positions = np.vstack([X.ravel(), Y.ravel(), Z.ravel()]).T
    kwargs = {'ctrlType': ctrlType}
    arg = [(position, kwargs) for position in positions]
    if rerunSims:
        sim_results = pool.map(sim_wrapper, arg)

        with open(ctrlType+"_sweep_results.pickle", "wb") as output_file:
                pickle.dump(sim_results, output_file,-1)
    else:
        with open(ctrlType+"_sweep_results.pickle", "rb") as fp:
            sim_results = pickle.load(fp)
    
    pool.close()


def results_to_ranges_and_plot(results_list):
    """
    Converts a results dict from scenarioDragRendezvous to a set of initial and final distance and speed errors, 
    and returns a plot of all of the Hill-frame trajectories.
    """
    fig = plt.figure()
    pos_errs = np.empty(len(results_list))
    vel_errs = np.empty(len(results_list))
    init_relpos = np.empty(len(results_list))
    init_relvel = np.empty(len(results_list))
    dens_list = np.empty(len(results_list))

    density_colorvals = np.logspace(-1,0.4,num=20)
    # normalizer = cmap.norma

    for ind, result in enumerate(results_list):
        hill_pos = result['relState.r_DC_H']
        hill_vel = result['relState.v_DC_H']
        init_relpos[ind] = np.linalg.norm(hill_pos[1,:])
        init_relvel[ind] = np.linalg.norm(hill_vel[1,:])
        pos_errs[ind] = np.linalg.norm(hill_pos[-1,:])
        vel_errs[ind] = np.linalg.norm(hill_vel[-1,:])
        dens_list[ind] = result['dens_mult']


        if ind%5==0:
            plt.plot(hill_pos[:,0],hill_pos[:,1] )
            plt.grid()
            plt.xlabel('Hill X (m)')
            plt.ylabel('Hill Y (m)')

    return init_relpos, init_relvel, pos_errs, vel_errs, dens_list, fig

def comparison_sweep(savePickle):

    with open("lqr_sweep_results.pickle", "rb") as fp:
        lqr_sim_results = pickle.load(fp)
    if not savePickle:
        os.remove("lqr_sweep_results.pickle")

    figlist = {}
    lqr_init_range, lqr_init_vel, lqr_err_range, lqr_err_vel, lqr_dens, lqr_fig = results_to_ranges_and_plot(lqr_sim_results)
    figlist[fileName+'_hillTrajectories'] = lqr_fig

    unique_ranges = np.unique(lqr_init_range.round(decimals=2))
    x_shape = len(unique_ranges)
    unique_dens = np.unique(lqr_dens)
    y_shape = len(unique_dens)

    import matplotlib.colors as colors
    fig = plt.figure()
    X,Y = np.meshgrid(unique_ranges, unique_dens)
    Z = lqr_err_range.reshape([x_shape, y_shape])
    Z_vel = lqr_err_vel.reshape([x_shape, y_shape])

    #   Position error contours
    contour = plt.contourf(X.T,Y.T,Z, 30,norm=colors.LogNorm(Z.min(), Z.max()))
    plt.ylabel('Density Multiplier')
    plt.xlabel('Initial Displacement (m)')
    cbar = fig.colorbar(contour)
    cbar.set_label("Terminal Positioning Error (m)")
    figlist[fileName+'_positionSensitivity'] = fig

    #   Velocity error contours
    fig2 = plt.figure()
    contour2 = plt.contourf(X.T,Y.T,Z_vel, 30,norm=colors.LogNorm(Z.min(), Z.max()))
    plt.ylabel('Density Multiplier')
    plt.xlabel('Initial Displacement (m)')
    cbar2 = fig2.colorbar(contour2)
    cbar2.set_label("Terminal Velocity Error (m/s)")
    figlist[fileName+'_velocitySensitivity'] = fig2

    return figlist

def run(show_plots, nuOffsetNum, densityNum, rerunSims=False, savePickle=False):
    drag_sensitivity_analysis('lqr', nuOffsetNum, densityNum, rerunSims=rerunSims)
    plots = comparison_sweep(savePickle)
    if show_plots:
        plt.show()
    return plots

if __name__=='__main__':
    nuOffsetNum = 15
    densityNum = 20
    run(True, nuOffsetNum, densityNum, rerunSims=True, savePickle=False)