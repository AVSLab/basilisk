from Basilisk.utilities import macros
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

class Plotter(object):
    def __init__(self, time_scale=macros.NANO2MIN, add_titles=True, name=""):
        self.plotter_name = name + "_"
        self.figure_names = list()
        self.time_scale = time_scale
        self.add_titles = add_titles
        self.color_list = self.set_color_list()
        return

    def set_color_list(self):
        color_list = ['deeppink', 'dodgerblue', 'lightgreen', 'salmon',
                      'm', 'gold', 'r', 'orange', 'darkblue', 'indigo']
        return color_list

    def create_figure(self, fig_name):
        fig = plt.figure(fig_name)
        fig.name = self.plotter_name + fig_name
        plt.xlabel('Time, min')
        if self.add_titles:
            plt.title(fig_name)
        self.figure_names.append(fig_name)
        return fig

    def show_plots(self):
        plt.show()

    def plot3components(self, vec):
        time = vec[:, 0] * macros.NANO2MIN
        for i in range(1, 4):
            plt.plot(time, vec[:, i], self.color_list[i])
        # plt.plot(time, vec[:, 1], self.color_list[0])
        # plt.plot(time, vec[:, 2], self.color_list[1])
        # plt.plot(time, vec[:, 3], self.color_list[2])
        return

    def plot4components(self, vec):
        time = vec[:, 0] * macros.NANO2MIN
        for i in range(1, 5):
            plt.plot(time, vec[:, i], self.color_list[-i])
        return

    def plot_norm_v3(self, vec, color):
        #print "plot_norm_v3()"
        time = vec[:, 0] * macros.NANO2MIN
        #print len(time)
        #print vec.shape[0]
        norm_vec = list()
        for i in range(0, len(time)):
            norm = np.linalg.norm(vec[i, 1:4])
            norm_vec.append(norm)
        #print len(norm_vec)
        plt.plot(time, norm_vec, color)

    # -- Attitude
    def plot_sigma(self, sigma):
        self.plot3components(sigma)
        plt.legend(['$\sigma_1$', '$\sigma_2$', '$\sigma_3$'])
        plt.ylabel('MRP')

    def plot_omega(self, omega):
        self.plot3components(omega)
        plt.ylabel('Angular Rate, rad/s')
        plt.legend(['$\omega_1$', '$\omega_2$', '$\omega_3$'])

    # -- Translation
    def plot_position(self, pos):
        self.plot3components(pos)
        plt.legend(['$r_1$', '$r_2$', '$r_3$'])
        plt.ylabel('Inertial Position, m')

    def plot_velocity(self, vel):
        self.plot3components(vel)
        plt.legend(['$v_1$', '$v_2$', '$v_3$'])
        plt.ylabel('Inertial Velocity, m/s')

    def plot_position_norm(self, pos):
        self.plot_norm_v3(pos, color=self.color_list[-1])
        plt.ylabel('Inertial Position Norm, m')

    def plot_velocity_norm(self, vel):
        self.plot_norm_v3(vel, color=self.color_list[-2])
        plt.ylabel('Inertial Velocity Norm, m/s')

    # -- Orbit
    def plot_3D_box(self, ax, rx_vec, ry_vec, rz_vec):
        max_range = np.array(
            [rx_vec.max() - rx_vec.min(), ry_vec.max() - ry_vec.min(), rz_vec.max() - rz_vec.min()]).max()
        Xb = 0.5 * max_range * np.mgrid[-1:2:2, -1:2:2, -1:2:2][0].flatten() + 0.5 * (rx_vec.max() + rx_vec.min())
        Yb = 0.5 * max_range * np.mgrid[-1:2:2, -1:2:2, -1:2:2][1].flatten() + 0.5 * (ry_vec.max() + ry_vec.min())
        Zb = 0.5 * max_range * np.mgrid[-1:2:2, -1:2:2, -1:2:2][2].flatten() + 0.5 * (rz_vec.max() + rz_vec.min())
        for xb, yb, zb in zip(Xb, Yb, Zb):
            ax.plot([xb], [yb], [zb], 'w')
        return

    def plot_nav_position_norm(self, vec):
        m2km = 1.0/1000.0
        time = vec[:, 0] * macros.NANO2MIN
        norm_vec = np.zeros_like(time)
        for i in range(0, vec.shape[0]):
            norm_vec[i] = np.linalg.norm(vec[i, 1:4]) * m2km
        plt.plot(time, norm_vec, self.color_list[0])
        plt.ylabel('Position Norm, k')


    def plot_orbit_xyz(self, fig, r_vec, color):
        m2km = 1.0 / 1000.0
        rx_vec = r_vec[:, 1] * m2km
        ry_vec = r_vec[:, 2] * m2km
        rz_vec = r_vec[:, 3] * m2km
        # fig = plt.figure(100, figsize=(3.75, 3.75))
        ax = fig.add_subplot(111, projection='3d')
        # ax.scatter(rx_vec[0], ry_vec[0], rz_vec[0], color=color)
        ax.plot(rx_vec, ry_vec, rz_vec, color=color)
        self.plot_3D_box(ax, rx_vec, ry_vec, rz_vec)
        ax.set_xlabel('$r_x$, km')
        ax.set_ylabel('$r_y$, km')
        ax.set_zlabel('$r_z$, km')
        return

    # -- Instruments
    def plot_miss_angle(self, miss_angle, color):
        time = miss_angle[:, 0] * macros.NANO2MIN
        miss_deg = miss_angle[:, 1] * macros.R2D
        plt.plot(time, miss_deg, color)
        plt.ylabel("Missed Angle, deg")
