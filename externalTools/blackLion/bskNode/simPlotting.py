''' '''
'''
 ISC License

 Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from Basilisk.utilities import macros as mc

plt.rcParams['figure.figsize'] = 5.0, 4.0
plt.rcParams.update({'font.size': 7})


# ------------------------------------------------- PLOT SAVING ------------------------------------------------- #
class PlotsSaver(object):
    def __init__(self, plots_path=None):
        self.plots_path = plots_path
        self.save_plots = False
        if self.plots_path:
            self.save_plots = True
        self.set_plot_params()

    def set_plot_params(self):
        plt.rcParams['figure.figsize'] = 5.0, 5.0
        plt.rcParams.update({'font.size': 7})

    def save_plot(self, plot_name):
        save_plot_in_path(path=self.plots_path, plot_name=plot_name)

def save_plot_in_path(path, plot_name):
    plt.savefig(path + "/" + plot_name + ".pdf")
    print('Saving Figure: %s' % (path + "/" + plot_name + ".pdf"))


# --------------------------------- COMPONENTS & SUBPLOT HANDLING ----------------------------------------------- #
color_x = 'dodgerblue'
color_y = 'salmon'
color_z = 'lightgreen'

color_1 = "gold"
color_2 = "orange"
color_3 = "r"
color_4 = "brown"

colors4_list = [color_1, color_2, color_3, color_4]


def plot3components(vec):
    time = vec[:, 0] * mc.NANO2MIN
    plt.xlabel('Time, min')
    plt.plot(time, vec[:, 1], color_x)
    plt.plot(time, vec[:, 2], color_y)
    plt.plot(time, vec[:, 3], color_z)


def plot4components(vec):
    time = vec[:, 0] * mc.NANO2MIN
    plt.xlabel('Time, min')
    for i in range(1, 4):
        plt.plot(time, vec[:, i], colors4_list[i-1])
    return


def plot_sigma(sigma):
    plot3components(sigma)
    plt.legend(['$\sigma_1$', '$\sigma_2$', '$\sigma_3$'])
    plt.ylabel('MRP')


def plot_omega(omega):
    plot3components(omega)
    plt.ylabel('Angular Rate, rad/s')
    plt.legend(['$\omega_1$', '$\omega_2$', '$\omega_3$'])


def subplot_sigma(subplot, sigma):
    plot3components(sigma)
    plt.legend(['$\sigma_1$', '$\sigma_2$', '$\sigma_3$'])
    plt.ylabel('MRP')


def subplot_omega(subplot, omega):
    plot3components(omega)
    plt.ylabel('Angular Rate, rad/s')
    plt.legend(['$\omega_1$', '$\omega_2$', '$\omega_3$'])


# ------------------------------------- MAIN PLOT HANDLING ------------------------------------------------------ #

def plot_trackingError(sigma_BR, omega_BR_B):
    fig = plt.figure()
    fig.suptitle("Attitude Tracking Error")#, fontsize=16)
    ax = plt.subplot("211")
    plot_sigma(sigma_BR)
    plt.ylim([-1, 1])
    ax = plt.subplot("212")
    plot_omega(omega_BR_B)
    return


def plot_attitudeGuidance(sigma_RN, omega_RN_N):
    fig = plt.figure()
    fig.suptitle("Attitude Reference")
    plt.subplot(211)
    plot_sigma(sigma_RN)
    plt.ylim([-1, 1])
    plt.ylim([-1.0, 1.0])
    plt.subplot(212)
    plot_omega(omega_RN_N)
    return


def plot_rotationalNav(sigma_BN, omega_BN_B):
    fig = plt.figure()
    fig.suptitle("Spacecraft Body Attitude")
    plt.subplot(211)
    plot_sigma(sigma_BN)
    plt.ylim([-1, 1])
    plt.subplot(212)
    plot_omega(omega_BN_B)
    return


def plot_controlTorque(Lr):
    plt.figure()
    plot3components(Lr)
    plt.ylabel('Control Torque, $N \cdot m$')
    plt.legend(['$L_{r,1}$', '$L_{r,2}$', '$L_{r,3}$'])
    plt.title('Control Torque $L_r$')
    return

def plot_wheels_torque_command(u_wheels):
    plt.figure()
    plot4components(u_wheels)
    plt.ylabel('Torque, $N \cdot m$')
    plt.legend(['$u_{1}$', '$u_{2}$', '$u_{3}$', '$u_{4}$'])
    plt.title('Wheel Torque Command $u$')

def show_plots():
    plt.show()
