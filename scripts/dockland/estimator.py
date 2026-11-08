"""Hedef konum kestirimi: ArduPilot PosVelEKF.cpp'nin Python portu + sabit hedef icin N/E filtresi."""
import numpy as np


class PosVelEKF:
    def __init__(self):
        self.x = np.zeros(2)            # [pos, vel]
        self.P = np.zeros(3)            # [P00, P01, P11]

    def init(self, pos, pos_var, vel, vel_var):
        self.x[:] = (pos, vel)
        self.P[:] = (pos_var, 0.0, vel_var)

    def predict(self, dt, dvel, dvel_noise):
        x, P = self.x, self.P
        self.x = np.array([x[0] + dt * x[1], x[1] + dvel])
        self.P = np.array([P[0] + 2 * dt * P[1] + dt * dt * P[2],
                           P[1] + dt * P[2],
                           P[2] + dvel_noise ** 2])

    def nis(self, pos, pos_var):
        r = pos - self.x[0]
        return r * r / (self.P[0] + pos_var)

    def fuse_pos(self, pos, pos_var):
        x, P = self.x, self.P
        S = P[0] + pos_var
        r = pos - x[0]
        k0, k1 = P[0] / S, P[1] / S
        self.x = np.array([x[0] + k0 * r, x[1] + k1 * r])
        self.P = np.array([P[0] * pos_var / S, P[1] * pos_var / S, P[2] - P[1] * P[1] / S])


class TargetFilter:
    """Sabit hedef: N/E icin 2 adet PosVelEKF. Durum = hedefin yerel NED konumu."""
    NIS_GATE = 9.0
    LOST_REINIT_S = 2.0

    def __init__(self):
        self.f = [PosVelEKF(), PosVelEKF()]
        self.ok = False
        self.t_last = 0.0
        self.t_pred = 0.0
        self.rejected = 0

    def predict(self, now):
        if not self.ok:
            return
        dt = now - self.t_pred
        self.t_pred = now
        for k in self.f:
            k.predict(dt, 0.0, 0.05 * dt)

    def update(self, now, tn, te, sigma):
        var = sigma * sigma
        if not self.ok or now - self.t_last > self.LOST_REINIT_S:
            self.f[0].init(tn, var, 0.0, 0.05)
            self.f[1].init(te, var, 0.0, 0.05)
            self.ok, self.t_pred = True, now
        else:
            if self.f[0].nis(tn, var) + self.f[1].nis(te, var) > 2 * self.NIS_GATE:
                self.rejected += 1
                return False
            self.f[0].fuse_pos(tn, var)
            self.f[1].fuse_pos(te, var)
        self.t_last = now
        return True

    def pos(self):
        return self.f[0].x[0], self.f[1].x[0]
