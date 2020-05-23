import matplotlib.pyplot as plt
from Basilisk.utilities import macros
from plotter import Plotter


class FSWPlotter(Plotter):
    def __init__(self, time_scale=macros.NANO2MIN, add_titles=True, plots_path=None, name="fsw"):
        super(FSWPlotter, self).__init__(time_scale, add_titles, name)
        self.plots_path = plots_path
        self.save_plots = False
        if self.plots_path:
            self.save_plots = True
            plt.rcParams['figure.figsize'] = 4.6, 3.2
            plt.rcParams.update({'font.size': 9})

    def save_plot(self, plot_name):
        plt.savefig(self.plots_path + "/" + plot_name + ".pdf", bbox_inches='tight', pad_inches=0)
        print('Saving Figure: %s' % (self.plots_path + "/" + plot_name + ".pdf", ))

    # Guidance: attitude reference
    def plot_sigmaRN(self, sigmaRN):
        fig = self.create_figure('sigmaRN')
        self.plot_sigma(sigmaRN)
        if self.save_plots:
            self.save_plot(plot_name=fig.name)
        return

    def plot_omegaRN_N(self, omega_RN_N):
        fig = self.create_figure('omega_RN_N')
        self.plot_omega(omega_RN_N)
        if self.save_plots:
            self.save_plot(plot_name=fig.name)
        return

    # Guidance: attitude tracking error
    def plot_sigmaBR(self, sigmaBR):
        fig = self.create_figure('sigmaBR')
        self.plot_sigma(sigmaBR)
        if self.save_plots:
            self.save_plot(plot_name=fig.name)
        return

    def plot_omegaBR_B(self, omega_BR_B):
        fig = self.create_figure('omega_BR_B')
        self.plot_omega(omega_BR_B)
        if self.save_plots:
            self.save_plot(plot_name=fig.name)
        return

    def plot_ref_attitude(self, sigma_RN, omega_RN_N):
        self.plot_sigmaRN(sigma_RN)
        self.plot_omegaRN_N(omega_RN_N)

    def plot_track_error(self, sigma_BR, omega_BR_B):
        self.plot_sigmaBR(sigma_BR)
        self.plot_omegaBR_B(omega_BR_B)

    def plot_rw_control(self, Lr, u_wheels):
        self.plot_control_Lr(Lr)
        self.plot_wheelTorques_u(u_wheels)

    # Controls: rw control torques
    def plot_control_Lr(self, Lr):
        fig = self.create_figure('Lr')
        self.plot3components(Lr)
        plt.legend(['$L_1$', '$L_2$', '$L_3$'])
        plt.ylabel('Control Torque, N$\cdot$m')
        if self.save_plots:
            self.save_plot(plot_name=fig.name)
        return

    def plot_wheelTorques_u(self, u_wheels):
        fig = self.create_figure('u_wheels')
        self.plot4components(u_wheels)
        plt.ylabel('Wheel Torques, N$\cdot$m')
        plt.legend(['$u_{1}$', '$u_{2}$', '$u_{3}$', '$u_{4}$'])
        if self.save_plots:
            self.save_plot(plot_name=fig.name)
        return
