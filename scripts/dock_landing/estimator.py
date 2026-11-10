"""Hedef konum kestirimi, sabit hedef icin N/E filtresi."""
import numpy as np

class PosVelEKF:
    def __init__(self):
        self.state = np.zeros(2)           # [konum, hız]
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

        print(self.state[0])

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
    NIS_GATE = 9.0       # ayrık değer eleme eşiği(ki-kare istatistiği) 
    LOST_REINIT_S = 2.0  # veri kesintisinde filtreyi sıfırlama zaman aşımı (sn)

    def __init__(self):
        # 0: KUZEY ekseni filtresi, 1: DOĞU ekseni filtresi
        self.filters = [PosVelEKF(), PosVelEKF()]
        self.is_initialized = False
        self.last_update_time = 0.0
        self.last_predict_time = 0.0
        self.rejected_count = 0

    def predict(self, current_time):
        if not self.is_initialized:
            return
        
        time_delta = current_time - self.last_predict_time
        self.last_predict_time = current_time

        # sabit hedef varsayımı, her iki eksen için de zaman tahmini
        for axis_filter in self.filters:
            axis_filter.predict(
                time_delta=time_delta,
                delta_velocity=0.0,
                velocity_noise=0.05 * time_delta
            )

    def update(self, current_time, target_north, target_east, measurement_sigma):
        measurement_variance = measurement_sigma ** 2

        # filtre başlamadıysa, yada uzun süredir veri alınmadıysa yeniden başlat
        if not self.is_initialized or current_time - self.last_update_time > self.LOST_REINIT_S:
            self.filters[0].initialize(
                target_north, measurement_variance, velocity=0.0, velocity_variance=0.05
            )
            self.filters[1].initialize(
                target_east, measurement_variance, velocity=0.0, velocity_variance=0.05
            )
            self.is_initialized    = True
            self.last_predict_time = current_time

        else:
            # NIS testi ile hatalı ölçümleri süz
            nis_north = self.filters[0].calculate_nis(target_north, measurement_variance)
            nis_east  = self.filters[1].calculate_nis(target_east, measurement_variance) 
            if (nis_north + nis_east) > (2 * self.NIS_GATE):
                self.rejected_count += 1
                return False

            # ölçümleri filtre durumuyla birleştir (fuse)
            self.filters[0].fuse_pos(target_north, measurement_variance)
            self.filters[1].fuse_pos(target_east, measurement_variance)

        self.last_update_time = current_time
        return True

    def get_position(self):
        """Güncel kuzey ve doğu konum kestirimlerini döndürür."""
        return self.filters[0].state[0], self.filters[1].state[0]
