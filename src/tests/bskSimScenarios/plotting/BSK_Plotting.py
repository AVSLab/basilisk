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

import matplotlib.pyplot as plt
import numpy as np
from Basilisk.utilities import macros as mc


# --------------------------------- COMPONENTS & SUBPLOT HANDLING ----------------------------------------------- #
color_x = 'dodgerblue'
color_y = 'salmon'
color_z = 'lightgreen'
m2km = 1.0 / 1000.0

def show_all_plots():
    plt.show()


def plot3components(vec):
    time = vec[:, 0] * mc.NANO2MIN
    plt.xlabel('Time, min')
    plt.plot(time, vec[:, 1], color_x)
    plt.plot(time, vec[:, 2], color_y)
    plt.plot(time, vec[:, 3], color_z)


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

# def plot_bodyTorque(Lb):
#     plt.figure()
#     plot3components(Lb)
#     plt.title('Body Torque $L_b$')
#     plt.ylabel('Body Torque, $N \cdot m$')
#     plt.legend(['$L_{b,1}$', '$L_{b,2}$', '$L_{b,3}$'])


def plot_controlTorque(Lr):
    plt.figure()
    plot3components(Lr)
    plt.ylabel('Control Torque, $N \cdot m$')
    plt.legend(['$L_{r,1}$', '$L_{r,2}$', '$L_{r,3}$'])
    plt.title('Control Torque $L_r$')
    return


def plot_trackingError(sigma_BR, omega_BR_B):
    plt.figure()
    plt.subplot(211)
    plot_sigma(sigma_BR)
    plt.title('Att Error: $\sigma_{BR}$')
    
    plt.subplot(212)
    #plt.figure()
    plot_omega(omega_BR_B)
    plt.title('Rate Error: $^B{\omega_{BR}}$')
    return


def plot_attitudeGuidance(sigma_RN, omega_RN_N):
    plt.figure()
    plt.subplot(211)
    plot_sigma(sigma_RN)
    plt.ylim([-1.0, 1.0])
    plt.title('Ref Att: $\sigma_{RN}$')
    
    plt.subplot(212)
    #plt.figure()
    plot_omega(omega_RN_N)
    plt.title('Ref Rate: $^N{\omega_{RN}}$')
    return


def plot_rotationalNav(sigma_BN, omega_BN_B):
    plt.figure()
    plt.subplot(211)
    plot_sigma(sigma_BN)
    plt.title('Sc Att: $\sigma_{BN}$')
    
    plt.subplot(212)
    #plt.figure()
    plot_omega(omega_BN_B)
    plt.title('Sc Rate: $^B{\omega_{BN}}$')
    return

def plot_shadow_factor(time, shadow_factor):
    plt.figure()
    outMat = np.array(shadow_factor).transpose()
    vector = outMat[1:].transpose()
    plt.plot(time, vector)
    plt.xlabel('Time min')
    plt.ylabel('Shadow Factor')
    plt.title("Eclipse Shadow Factor")
    return

def plot_sun_point(time, sunPoint):
    plt.figure()
    plt.xlabel('Time')
    plt.ylabel('Sun Point Vec')
    plot3components(sunPoint)
    plt.title('Sun Point Vector')
    return


def plot_orbit(r_BN):
    plt.figure()
    plt.xlabel('$R_x$, km')
    plt.ylabel('$R_y$, km')
    plt.plot(r_BN[:, 1] * m2km, r_BN[:, 2] * m2km, color_x)
    plt.scatter(0, 0)
    plt.title('Spacecraft Orbit')
    return



