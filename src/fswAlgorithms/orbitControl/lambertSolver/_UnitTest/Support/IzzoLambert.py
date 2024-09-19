
## Documentation is Izzo's 2015 paper "Revisiting Lambert's Problem"
# cpp source https://github.com/esa/pykep/blob/master/src/lambert_problem.cpp

import numpy as np

class IzzoSolve():
    def __init__(self, r1, r2, tof, mu, max_multiRev = 10, max_iter = 12):
        self.mu = mu
        self.max_multiRev = max_multiRev
        self.max_iter = max_iter
        self.tof = tof
        self.r1 = r1
        self.r2 = r2
        assert self.mu > 0, print("Mu must be positive")
        assert self.tof > 0, print("ToF must be positive")

        self.c = self.r2 - self.r1
        R1 = np.linalg.norm(r1)
        R2 = np.linalg.norm(r2)
        c_norm = np.linalg.norm(self.c)

        self.s = 0.5*(R1 + R2 + c_norm)
        self.r1_hat = self.r1/R1
        self.r2_hat = self.r2/R2
        h_hat = np.cross(self.r1_hat, self.r2_hat)/np.linalg.norm(np.cross(self.r1_hat, self.r2_hat))
        assert h_hat[2]!= 0 , print("r1 and r2 don't define a plane")

        self.lambda2 = 1. - c_norm / self.s
        self.lambd = np.sqrt(self.lambda2)

        self.lambd = np.sign(h_hat[2])*self.lambd
        self.t1_hat = -np.sign(h_hat[2])*np.cross(self.r1_hat, h_hat)
        self.t2_hat = -np.sign(h_hat[2])*np.cross(self.r2_hat, h_hat)

        self.lambda3 = self.lambd ** 3.
        self.T = np.sqrt(2. * self.mu / self.s ** 3) * self.tof

        self.solve()

    def solve(self):
        x_list = self.findxy(self.lambd, self.T)
        self.x = x_list
        gamma = np.sqrt(self.mu * self.s / 2.)
        rho = (np.linalg.norm(self.r1) - np.linalg.norm(self.r2)) / np.linalg.norm(self.c)
        sigma = np.sqrt(1. - rho ** 2)

        self.v1 = np.zeros([len(x_list), 3])
        self.v2 = np.zeros([len(x_list), 3])

        for i in range(len(x_list)):
            y = np.sqrt(1. - self.lambda2 + self.lambda2 * x_list[i] ** 2)
            vr1 = gamma * ((self.lambd * y - x_list[i]) - rho * (self.lambd * y + x_list[i])) / np.linalg.norm(self.r1)
            vr2 = -gamma * ((self.lambd * y - x_list[i]) + rho * (self.lambd * y + x_list[i])) / np.linalg.norm(self.r2)
            vt = gamma * sigma * (y + self.lambd * x_list[i])
            vt1 = vt / np.linalg.norm(self.r1)
            vt2 = vt / np.linalg.norm(self.r2)

            self.v1[i, :] = vr1 * self.r1_hat + vt1 * self.t1_hat
            self.v2[i, :] = vr2 * self.r2_hat + vt2 * self.t2_hat

    def findxy(self, lambd, T):
        M_max = np.floor(T/np.pi)
        T_00 = np.arccos(lambd) + lambd*np.sqrt(1. - lambd**2)
        T_0 = T_00 + M_max * np.pi
        if T < T_0 and M_max > 0:
            x_old = 0.
            x_new = 0.
            T_min = T_0
            for it in range(12):
                DT, D2T, D3T = self.dTdx(x_old, T_min)
                if DT != 0.:
                    x_new = x_old - 2. * DT * D2T / (2. * D2T ** 2 - DT * D3T)
                err = np.abs(x_old - x_new)
                if err < 1E-13:
                    break
                self.tof = self.x2tof(x_new, M_max)
                T_min = self.tof
                x_old = x_new
            if T_min > T:
                M_max -= 1

        T_1 = 2. / 3. * (1. - self.lambd ** 3)
        M_max = np.min([M_max, self.max_multiRev])
        x = np.zeros([2*int(M_max) + 1])
        iters = np.zeros([2*int(M_max) + 1])
        if T >= T_00:
            self.x0 = -(T - T_00) / (T - T_00 + 4)
        elif T <= T_1:
            self.x0 = 5./2. * T_1 * (T_1 - T) / (T*(1 - self.lambd ** 5)) + 1
        else:
            self.x0 = (T_00 / T) ** (np.log(T_1 / T_00)/np.log(2)) - 1.
        iters[0] = self.householder(T, self.x0, 0, 1e-5, self.max_iter)
        x[0] = self.x0
        for i in range(1, int(M_max) + 1):
            tmp = ((i * np.pi + np.pi) / (8.0 * T)) ** (2. / 3.)
            x[2*i - 1] = (tmp - 1) / (tmp + 1)
            iters[2*i - 1] = self.householder(T, x[2*i - 1], i, 1e-8, self.max_iter)
            x[2*i - 1] = self.x0

            tmp = ((8.0 * T) / (i * np.pi)) ** (2. / 3.)
            x[2*i] = (tmp - 1) / (tmp + 1)
            iters[2*i] = self.householder(T, x[2*i], i, 1e-8, self.max_iter)
            x[2*i] = self.x0
        return x

    def get_v1(self):
        return self.v1

    def get_v2(self):
        return self.v2

    def dTdx(self, x, T):
        umx2 = 1. - x ** 2
        y = np.sqrt(1. - self.lambda2 * umx2)

        DT = 1. / umx2 * (3. * T * x - 2. + 2. * self.lambda3 * x / y)
        D2T = 1. / umx2 * (3. * T + 5. * x * DT + 2. * (1. - self.lambda2) * self.lambda3 / y**3)
        D3T = 1. / umx2 * (7. * x * D2T + 8. * DT - 6. * (1. - self.lambda2) * self.lambda2 * self.lambda3 * x / y**5)

        return DT, D2T, D3T

    def x2tof2(self, x, N):
        a = 1.0 / (1.0 - x ** 2)
        if a > 0: #Eclipse case
            alfa = 2.0 * np.arccos(x)
            beta = 2.0 * np.arcsin(np.sqrt(self.lambda2 / a))
            if self.lambd < 0.0:
                beta = -beta
            tof = ((a * np.sqrt(a) * ((alfa - np.sin(alfa)) - (beta - np.sin(beta)) + 2.0 * np.pi * N)) / 2.0)
        else:
            alfa = 2.0 * np.arccosh(x)
            beta = 2.0 * np.arcsinh(np.sqrt(-self.lambda2 / a))
            if self.lambd < 0.0:
                beta = -beta
            tof = (-a * np.sqrt(-a) * ((beta - np.sinh(beta)) - (alfa - np.sinh(alfa))) / 2.0)
        return tof

    def x2tof(self, x, N):
        battin = 0.01
        lagrange = 0.2
        dist = np.abs(x - 1)
        if dist < lagrange and dist > battin: # Use langrange formulation
            return self.x2tof2(x, N)
        K = self.lambda2
        E = x ** 2 - 1.
        rho = np.abs(E)
        z = np.sqrt(1 + K * E)
        if dist < battin: # Use Battin formulation
            eta = z - self.lambd * x
            S1 = 0.5 * (1.0 - self.lambd - x * eta)
            Q = self.hypergeometricF(S1, 1e-11)
            Q = 4.0 / 3.0 * Q
            tof = (eta ** 3 * Q + 4.0 * self.lambd * eta) / 2.0 + N * np.pi / rho ** 1.5
        else:  # Use Lancaster formulation
            y = np.sqrt(rho)
            g = x * z - self.lambd * E
            if E < 0:
                l = np.arccos(g)
                d = N * np.pi + l
            else:
                f = y * (z - self.lambd * x)
                d = np.log(f + g)
            tof = (x - self.lambd * z - d / y) / E
        return tof

    def householder(self, T, x0, N, eps, iter_max):
        for it in range(iter_max):
            tof = self.x2tof(x0, N)
            DT, DDT, DDDT = self.dTdx(x0, tof)
            delta = tof - T
            DT2 = DT ** 2
            xnew = x0 - delta * (DT2 - delta * DDT / 2.0) / (DT * (DT2 - delta * DDT) + DDDT * delta * delta / 6.0)
            err = np.abs(x0 - xnew)
            x0 = xnew
            self.x0 = xnew
            if err < eps:
                break
        return it

    def hypergeometricF(self, z, tol):
        Sj = 1.
        Cj = 1.
        err = 1.
        for j in range(12):
            Cj1 = Cj * (3. + j) * (1.0 + j) / (2.5 + j) * z / (j + 1)
            Sj1 = Sj + Cj1
            err = np.abs(Cj1)
            Sj = Sj1
            Cj = Cj1
            if err < tol:
                break
        return Sj
