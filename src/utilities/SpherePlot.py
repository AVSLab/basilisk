import copy

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from Basilisk.utilities import RigidBodyKinematics


def plotSpheres(posDataL_N, posDataF_N, attDataL_N, attDataF_N, spPosListLeader_H, rListLeader,
               LeaderSpCharges, spPosListFollower_H, rListFollower, FollowerSpCharges):

    figureList = {}
    plt.figure(3)
    ax = plt.axes(projection='3d')
# get S/C position
    r_LN_N = np.array([0., 0., 0.])
    r_FL_N = posDataF_N[0, 0:3] - posDataL_N[0, 0:3]
    # get sphere locations
    dcm_NL = RigidBodyKinematics.MRP2C(attDataL_N[0, 0:3]).transpose()
    dcm_NF = RigidBodyKinematics.MRP2C(attDataF_N[0, 0:3]).transpose()
    spPosL_H = np.dot(dcm_NL, np.array(spPosListLeader_H).transpose()).transpose()
    spPosF_H = np.dot(dcm_NF,   np.array(spPosListFollower_H).transpose()).transpose()
    radiiL = copy.deepcopy(rListLeader)
    radiiF = copy.deepcopy(rListFollower)
    # Define to plot spheres
    u = np.linspace(0, np.pi, 10)
    v = np.linspace(0, 2 * np.pi, 10)
    x = np.outer(np.sin(u), np.sin(v))
    y = np.outer(np.sin(u), np.cos(v))
    z = np.outer(np.cos(u), np.ones_like(v))

    # Find the minimum and maximum charges in order to normalize the values
    all_charges = np.array(LeaderSpCharges + FollowerSpCharges)
    max_q = max(all_charges)
    min_q = min(all_charges)

    # Plot the spheres for the Leader spacecraft
    for ii in range(0, len(radiiL)):
        r_SpN_N = r_LN_N + spPosL_H[ii, 0:3]
        # If both spacecraft are negatively charged
        if max_q < 0 and min_q < 0:
            # Normalize the sphere charge
            # normcl = (LeaderSpCharges[ii] - min_q) / (0 - min_q)
            normcl = (LeaderSpCharges[ii] - 0) / (min_q - 0)
            # Use green colormap to denote negative charge
            cmap = plt.get_cmap("Greens")
            color = cmap(normcl)
            norm = mpl.colors.Normalize(vmin=0, vmax=min_q)
        # If both spacecraft are positively charged
        elif max_q > 0 and min_q > 0:
            normcl = (LeaderSpCharges[ii] - 0) / (max_q - 0)
            # Use red colormap to denote positive charge
            cmap = plt.get_cmap("Reds")
            color = cmap(normcl)
            norm = mpl.colors.Normalize(vmin=0, vmax=max_q)
        # If the spacecraft have opposing charges
        elif max_q > 0 and min_q < 0:
            normcl = (LeaderSpCharges[ii] - min_q) / (max_q - min_q)
            # Use yellow to denote a neutral charge
            cmap = plt.get_cmap("RdYlGn")
            color = cmap(normcl)
            norm = mpl.colors.Normalize(vmin=min_q, vmax=max_q)
        else:
            cmap = plt.get_cmap('Greys')
            color = (.5, .5, .5)
            norm = mpl.colors.Normalize(vmin=min_q, vmax=max_q)

        ax.plot_surface(r_SpN_N[0] + radiiL[ii] * x, r_SpN_N[1] + radiiL[ii] * y, r_SpN_N[2] + radiiL[ii] * z, color=color)

    for ii in range(0, len(radiiF)):
        r_SpN_N = r_LN_N + r_FL_N + spPosF_H[ii, 0:3]
        if max_q < 0 and min_q < 0:
            normcf = (FollowerSpCharges[ii] - min_q) / (0 - min_q)
            cmap = plt.get_cmap("Greens_r")
            color = cmap(normcf)
            norm = mpl.colors.Normalize(vmin=min_q, vmax=0)
        elif max_q > 0 and min_q > 0:
            normcf = (FollowerSpCharges[ii] - 0) / (max_q - 0)
            cmap = plt.get_cmap("Reds")
            color = cmap(normcf)
            norm = mpl.colors.Normalize(vmin=0, vmax=max_q)
        elif max_q > 0 and min_q < 0:
            normcf = (FollowerSpCharges[ii] - min_q) / (max_q - min_q)
            cmap = plt.get_cmap("RdYlGn")
            color = cmap(normcf)
            norm = mpl.colors.Normalize(vmin=min_q, vmax=max_q)
        else:
            cmap = plt.get_cmap('Greys')
            color = (.5, .5, .5)
            norm = mpl.colors.Normalize(vmin=min_q, vmax=max_q)

        ax.plot_surface(r_SpN_N[0] + radiiF[ii] * x, r_SpN_N[1] + radiiF[ii] * y, r_SpN_N[2] + radiiF[ii] * z, color=color)

    limits = np.array([getattr(ax, f'get_{axis}lim')() for axis in 'xyz'])
    ax.set_box_aspect(np.ptp(limits, axis=1))
    ax.set_xlabel('Radial')
    ax.set_ylabel('Along Track')
    ax.set_zlabel('Direction of Angular Momentum')
    plt.title('MSM Representation')

    figureList['Charged_Spheres'] = plt.figure(3)

    # Add color map based on the spacecraft charges
    fig2, ax = plt.subplots(figsize=(6, 1))
    fig2.colorbar(mpl.cm.ScalarMappable(norm=norm, cmap=cmap),
                  cax=ax, orientation='horizontal', label='Sphere Charge (C)')

    figureList['Colorbar'] = plt.figure(4)

    return figureList


def NormalizeData(data):
    return (data - np.min(data)) / (np.max(data) - np.min(data))

