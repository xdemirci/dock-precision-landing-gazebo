"""Hedef konum kestirimi, sabit hedef icin N/E filtresi."""
import numpy as np

class PosVelEKF:
    def __init__(self):
        self.state = np.zeros(2)      # [konum, hız]
        self.covariance = np.zeros(3)      # [P00, P01, P11], kovaryans matrisi elemanları

    def initialize(self, posisiton, position_variance, velocity, velocity_variance):
        self.state[:] = (posisiton, velocity)
        self.covariance[:] = (position_variance, 0.0, velocity_variance)

    def predict(self, time_delta, delta_velocity, velocity_noise):
        state, covariance = self.state, self.covariance

        # konum ve hız tahmini
        self.state = np.array([
            state[0] + time_delta * state[1],
            state[1] + delta_velocity
        ])

        # kovaryans (hata belrsizliği) tahmini
        self.covariance = np.array([
            covariance[0] + 2 * time_delta * covariance[1] + time_delta * time_delta * covariance[2],
            covariance[1] + time_delta * covariance[2],
            covariance[2] + velocity_noise ** 2
        ])

    def calculate_nis(self, measured_position, position_variance): # Normalized Innovation Squared
        residual = measured_position - self.state[0]
        innovation_variance = self.covariance[0] + position_variance
        return (residual ** 2) / innovation_variance

    def fuse_pos(self, measured_position, position_variance):
        state, covariance = self.state, self.covariance
        innovation_variance = covariance[0] + position_variance
        residual = measured_position - state[0]
        
        # kalman kazançları (konum ve hız)
        gain_position = covariance[0] / innovation_variance
        gain_velocity = covariance[1] / innovation_variance
        
        # durum güncellemesi
        self.state = np.array([
            state[0] + gain_position * residual,
            state[1] + gain_velocity * residual
        ])
        
        # kovaryans güncellemesi
        self.covariance = np.array([
            covariance[0] * position_variance / innovation_variance,
            covariance[1] * position_variance / innovation_variance,
            covariance[2] - (covariance[1] ** 2) / innovation_variance
        ])


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
            self.f[0].initialize(tn, var, 0.0, 0.05)
            self.f[1].initialize(te, var, 0.0, 0.05)
            self.ok, self.t_pred = True, now
        else:
            if self.f[0].calculate_nis(tn, var) + self.f[1].calculate_nis(te, var) > 2 * self.NIS_GATE:
                self.rejected += 1
                return False
            self.f[0].fuse_pos(tn, var)
            self.f[1].fuse_pos(te, var)
        self.t_last = now
        return True

    def pos(self):
        return self.f[0].state[0], self.f[1].state[0]
